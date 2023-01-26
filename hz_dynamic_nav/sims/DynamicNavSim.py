from typing import List, Optional

import habitat_sim
import magnum as mn
import numpy as np
from habitat.core.registry import registry
from habitat.core.simulator import Observations
from habitat.sims.habitat_simulator import sim_utilities
from habitat.sims.habitat_simulator.habitat_simulator import HabitatSim
from hz_dynamic_nav.utils.geometry_utils import get_heading_error, quat_to_rad
from omegaconf import DictConfig


@registry.register_simulator(name="DynamicNav")
class DynamicNavSim(HabitatSim):
    def __init__(self, config: DictConfig) -> None:
        super().__init__(config=config)
        obj_templates_mgr = self.get_object_template_manager()
        self.people_template_ids = obj_templates_mgr.load_configs(
            "/nethome/jspaeth7/home-flash/workspaces/habitat-lab/habitat-baselines/data/person_meshes"
        )
        self.person_ids = []
        self.people_mask = config.get("PEOPLE_MASK", False)
        self.num_people = config.get("NUM_PEOPLE", 1)
        self.social_nav = True
        self.interactive_nav = False

        # People params
        self.people_mask = config.get("PEOPLE_MASK", False)
        self.lin_speed = config.PEOPLE_LIN_SPEED
        self.ang_speed = np.deg2rad(config.PEOPLE_ANG_SPEED)
        self.time_step = config.TIME_STEP

    def reset_people(self):
        agent_position = self.get_agent_state().position

        # Check if humans have been erased (sim was reset)
        if not sim_utilities.get_all_object_ids(self):
            self.person_ids = []
            for _ in range(self.num_people):
                for person_template_id in self.people_template_ids:
                    self.person_ids.append(self.add_object(person_template_id))

        # Spawn humans
        min_path_dist = 3
        max_level = 0.6
        agent_x, agent_y, agent_z = self.get_agent_state(0).position
        self.people = []
        for person_id in self.person_ids:
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
            self.set_translation([start[0], start[1] + 0.9, start[2]], person_id)
            self.set_rotation(rotation, person_id)
            self.set_object_motion_type(
                habitat_sim.physics.MotionType.KINEMATIC, person_id
            )
            spf = ShortestPathFollowerv2(
                sim=self,
                object_id=person_id,
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
        for person_id in sim_utilities.get_all_object_ids(self):
            pos = self.get_translation(person_id)
            all_pos.append(pos)
            self.set_translation([pos[0], pos[1] + 10, pos[2]], person_id)

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
        for pos, person_id in zip(all_pos, sim_utilities.get_all_object_ids(self)):
            self.set_translation(pos, person_id)

        return observations


class ShortestPathFollowerv2:
    def __init__(
        self,
        sim: DynamicNavSim,
        object_id,  # int for old Habitat, something else for new Habitat...
        waypoints: List[np.ndarray],
        lin_speed: float,
        ang_speed: float,
        time_step: float,
    ):
        self._sim = sim
        self.object_id = object_id

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
        self.max_linear_vel = np.random.rand() * (0.1) + self.lin_speed - 0.1

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
        translation = self._sim.get_translation(self.object_id)
        magnum_quaternion = self._sim.get_rotation(self.object_id)

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

        self._sim.set_translation(rigid_state.translation, self.object_id)
        self._sim.set_rotation(rigid_state.rotation, self.object_id)
        self.current_position = rigid_state.translation
