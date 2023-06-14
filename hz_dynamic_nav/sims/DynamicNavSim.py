import math
from typing import List, Optional

import cv2 as cv
import habitat
import habitat_sim
import magnum as mn
import numpy as np
from habitat.core.registry import registry
from habitat.core.simulator import Observations
from habitat.sims.habitat_simulator.habitat_simulator import HabitatSim
from hz_dynamic_nav.utils.geometry_utils import get_heading_error, quat_to_rad
from omegaconf import DictConfig

PEOPLE_HEIGHT_OFFSET = 0.9


@registry.register_simulator(name="DynamicNav")
class DynamicNavSim(HabitatSim):
    def __init__(self, config: DictConfig) -> None:
        super().__init__(config=config)
        obj_templates_mgr = self.get_object_template_manager()
        self.people_template_ids = obj_templates_mgr.load_configs(
            "/nethome/jspaeth7/home-flash/workspaces/habitat-lab/habitat-baselines/data/person_meshes"
        )
        self.person_ids = []
        self.social_nav = True
        self.interactive_nav = False

        # People params
        self.people_mask = config.people_mask
        self.people_spawn_type = config.people_spawn_type
        self.people_density = config.people_density
        self._people_num = config.people_num
        self.lin_speed = config.people_lin_speed
        self.ang_speed = np.deg2rad(config.people_ang_speed)
        self.people_stop_distance = config.people_stop_distance
        self.time_step = config.time_step

        assert self.people_spawn_type in ["density", "num"], "Invalid people spawn type"

    @property
    def people_num(self) -> int:
        """
        Get number of people to spawn based on spawn type.
        Careful calling this if spawn type is density, as it will recalculate scene area every time,
            which could be expensive.
        :return:
        """
        map_resolution = 256
        if self.people_spawn_type == "density":
            # Get obstacle map of current level as np array. 0 occupied, 1 unoccupied.
            topdown_map = habitat.utils.visualizations.maps.get_topdown_map_from_sim(
                sim=self, map_resolution=map_resolution, draw_border=False
            )

            # Get area of unoccupied space in pixels, convert to meters
            unoccupied_pixels = np.sum(topdown_map)  # Sum method
            meters_per_pixel = (
                habitat.utils.visualizations.maps.calculate_meters_per_pixel(
                    map_resolution=map_resolution, sim=self
                )
            )
            sq_meters_per_pixel = meters_per_pixel**2
            sq_meters = unoccupied_pixels * sq_meters_per_pixel
            people_spawn_num = int(self.people_density * sq_meters)
            return people_spawn_num

        else:
            return self._people_num

    def reset_people(self):
        agent_position = self.get_agent_state().position
        rigid_object_manager = self.get_rigid_object_manager()
        # Check if humans have been erased (sim was reset)
        if rigid_object_manager.get_num_objects() == 0:

            # Get number of people.
            self.person_references = []
            for _ in range(self.people_num):
                person_template_id = self.people_template_ids[0]
                person_reference = rigid_object_manager.add_object_by_template_id(
                    person_template_id
                )
                self.person_references.append(person_reference)

        # Spawn humans
        min_path_dist = 3
        max_level = 0.6
        agent_x, agent_y, agent_z = self.get_agent_state(0).position
        self.people = []
        for person_reference in self.person_references:
            valid_walk = False
            while not valid_walk:
                start = np.array(self.sample_navigable_point())
                goal = np.array(self.sample_navigable_point())
                distance = self.geodesic_distance(start, goal)
                valid_distance = distance > min_path_dist

                # Make sure paths don't span multiple floors
                valid_level = (
                    abs(start[1] - agent_position[1]) < max_level
                    and abs(goal[1] - agent_position[1]) < max_level
                )

                sp = habitat_sim.nav.ShortestPath()
                sp.requested_start = start
                sp.requested_end = goal
                found_path = self.pathfinder.find_path(sp)

                # Don't spawn people too close to the robot
                valid_start = (
                    np.sqrt((start[0] - agent_x) ** 2 + (start[2] - agent_z) ** 2) > 0.5
                )

                valid_walk = (
                    valid_distance and valid_level and found_path and valid_start
                )
                if not valid_distance:
                    min_path_dist *= 0.95
            waypoints = sp.points
            heading = np.random.rand() * 2 * np.pi - np.pi
            rotation = np.quaternion(np.cos(heading), 0, np.sin(heading), 0)
            rotation = np.normalized(rotation)
            rotation = mn.Quaternion(rotation.imag, rotation.real)
            person_reference.translation = mn.Vector3(
                start[0], start[1] + PEOPLE_HEIGHT_OFFSET, start[2]
            )
            person_reference.rotation = rotation
            person_reference.motion_type = habitat_sim.physics.MotionType.KINEMATIC
            spf = ShortestPathFollowerv2(
                sim=self,
                person_reference=person_reference,
                waypoints=waypoints,
                lin_speed=self.lin_speed,
                ang_speed=self.ang_speed,
                time_step=self.time_step,
            )
            self.people.append(spf)

    def get_observations_at(
        self,
        position: Optional[List[float]] = None,
        rotation: Optional[List[float]] = None,
        keep_agent_at_new_pose: bool = False,
    ) -> Optional[Observations]:
        observations = super().get_observations_at(
            position,
            rotation,
            keep_agent_at_new_pose,
        )

        if observations is None:
            return None

        if not self.people_mask:
            return observations

        """
        Get pixels of just people
        """
        # 'Remove' people
        all_pos = []
        for person_reference in self.person_references:
            translation = person_reference.translation
            all_pos.append(translation)
            person_reference.translation[1] = translation[1] + 10

        # Refresh observations
        no_ppl_observations = super().get_observations_at(
            position=position,
            rotation=rotation,
            keep_agent_at_new_pose=True,
        )

        # Remove non-people pixels
        observations["people"] = observations["depth"].copy()
        observations["people"][
            observations["people"] == no_ppl_observations["depth"]
        ] = 0

        # Put people back
        for pos, person_reference in zip(all_pos, self.person_references):
            person_reference.translation = pos

        return observations


class ShortestPathFollowerv2:
    def __init__(
        self,
        sim: DynamicNavSim,
        person_reference,  # int for old Habitat, something else for new Habitat...
        waypoints: List[np.ndarray],
        lin_speed: float,
        ang_speed: float,
        time_step: float,
    ):
        self._sim = sim
        self.person_reference = person_reference

        self.vel_control = habitat_sim.physics.VelocityControl()
        self.vel_control.controlling_lin_vel = True
        self.vel_control.controlling_ang_vel = True
        self.vel_control.lin_vel_is_local = True
        self.vel_control.ang_vel_is_local = True

        # Given waypoints from 0 to N, we want to cyclically go from 0 to N and
        # then from N to 1 (and on and on). So we add the reverse of the
        # waypoints to the end of the list, without the first or last element.
        self.waypoints = list(waypoints) + list(waypoints)[::-1][1:-1]

        self.next_waypoint_idx = 1
        self.done_turning = False
        self.current_position = waypoints[0]

        # People params
        self.lin_speed = lin_speed
        self.ang_speed = ang_speed
        self.time_step = time_step
        # Scale noise to be 40% of max speed.
        # 0.4 comes from original 0.1 multiplier with default speed of 0.25 (0.1/0.25 = 0.4)
        lin_vel_noise = (np.random.rand() - 0.5) * (2) * (self.lin_speed * 0.4)
        self.max_linear_vel = self.lin_speed + lin_vel_noise

    def step(self):
        """
        Step the shortest path follower. Objects will be moved in a point-turn
        manner (i.e. they won't move forward until they have turned to face
        the next waypoint).
        :return:
        """
        # If waypoint_idx exceeds max length, wrap around
        waypoint_idx = self.next_waypoint_idx % len(self.waypoints)

        waypoint = np.array(self.waypoints[waypoint_idx])
        translation = self.person_reference.translation
        magnum_quaternion = self.person_reference.rotation

        # Face the next waypoint if we aren't already facing it
        if not self.done_turning:
            # Get current global heading
            numpy_quaternion = np.quaternion(
                magnum_quaternion.scalar, *magnum_quaternion.vector
            )
            heading = -quat_to_rad(numpy_quaternion) + np.pi / 2

            # Get heading necessary to face next waypoint
            # In habitat, syntax is (x, z, y), for some reason...
            theta = math.atan2(
                waypoint[2] - translation[2], waypoint[0] - translation[0]
            )
            theta_diff = get_heading_error(heading, theta)
            direction = 1 if theta_diff < 0 else -1

            # If turning at max speed for the entire time step would overshoot,
            # only turn at the speed necessary to face the waypoint by the end
            # of the time step. Added a buffer of 20% percent to avoid
            # very small pivots that waste time.
            if self.ang_speed * self.time_step * 1.2 >= abs(theta_diff):
                angular_velocity = -theta_diff / self.time_step
                self.done_turning = True
            else:
                angular_velocity = self.ang_speed * direction

            self.vel_control.linear_velocity = np.zeros(3)
            self.vel_control.angular_velocity = np.array([0.0, angular_velocity, 0.0])

        # If we ARE facing the next waypoint, then move forward
        else:
            # If next move would normally overshoot, move just the right amount
            distance = np.sqrt(
                (translation[0] - waypoint[0]) ** 2
                + (translation[2] - waypoint[2]) ** 2
            )

            # If moving forward at max speed for the entire time step would
            # overshoot, only move at the speed necessary to reach the waypoint
            # by the end of the time step. Added a buffer of 20% percent to
            # avoid very small moves that waste time.
            if self.max_linear_vel * self.time_step * 1.2 >= distance:
                linear_velocity = distance / self.time_step
                self.done_turning = False  # start turning to next waypoint
                self.next_waypoint_idx += 1
            else:
                linear_velocity = self.max_linear_vel

            self.vel_control.angular_velocity = np.zeros(3)
            self.vel_control.linear_velocity = np.array([0.0, 0.0, linear_velocity])

        rigid_state = habitat_sim.bindings.RigidState(magnum_quaternion, translation)
        rigid_state = self.vel_control.integrate_transform(self.time_step, rigid_state)

        orig_translation = np.array(rigid_state.translation)
        rigid_state.translation = self._sim.pathfinder.snap_point(
            rigid_state.translation
        )

        if np.isnan(rigid_state.translation[1]):
            rigid_state.translation = orig_translation
        else:
            rigid_state.translation[1] += PEOPLE_HEIGHT_OFFSET

        # Check if updating the location of the person would result in a collision
        # with the agent. If so, don't update the person's location.
        agent_pos = self._sim.get_agent_state().position
        distance = np.sqrt(
            (rigid_state.translation[0] - agent_pos[0]) ** 2
            + (rigid_state.translation[2] - agent_pos[2]) ** 2
        )

        if distance > self._sim.people_stop_distance:
            self.person_reference.translation = rigid_state.translation
            self.person_reference.rotation = rigid_state.rotation
            self.current_position = rigid_state.translation
