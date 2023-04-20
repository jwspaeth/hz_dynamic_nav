from typing import TYPE_CHECKING, Any, Dict, List, Optional, Sequence, Tuple, Union

import habitat_sim
import magnum as mn
import numpy as np
from gym import spaces
from habitat.core.embodied_task import EmbodiedTask, SimulatorTaskAction
from habitat.core.registry import registry
from habitat.core.simulator import ActionSpaceConfiguration
from habitat.core.spaces import ActionSpace, EmptySpace
from habitat.sims.habitat_simulator.actions import HabitatSimActions
from habitat.tasks.nav.nav import NavigationEpisode, StopAction
from habitat_sim import RigidState
from habitat_sim._ext.habitat_sim_bindings import VelocityControl
from hz_dynamic_nav.actions.RecursiveSimulatorTaskAction import (
    RecursiveSimulatorTaskAction,
)


@registry.register_task_action
class DiscreteVelocitySingleAction(SimulatorTaskAction):
    name = "discrete_velocity_single_action"

    def __init__(self, linear_velocity, angular_velocity, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

        self.vel_control = VelocityControl()
        self.vel_control.controlling_lin_vel = True
        self.vel_control.controlling_ang_vel = True
        self.vel_control.lin_vel_is_local = True
        self.vel_control.ang_vel_is_local = True

        self.min_abs_lin_speed = self._config.min_abs_lin_speed
        self.min_abs_ang_speed = self._config.min_abs_ang_speed
        self.time_step = self._config.time_step

    @property
    def action_space(self):
        # Needed to override parent method
        return EmptySpace()

    def reset(
        self, task: EmbodiedTask, episode: NavigationEpisode, *args: Any, **kwargs: Any
    ):
        task.is_stop_called = False

    def step(self, task: EmbodiedTask, **kwargs):
        r"""Moves the agent with a provided linear and angular velocity for the
        provided amount of time

        Args:
            action: dict containing the discrete action id, which then maps to lin and ang velocities
                with velocity_dict
        """
        final_pos, final_rot, backwards, collided, task.is_stop_called = self.teleport()
        agent_observations = self._sim.get_observations_at(
            position=final_pos,
            rotation=final_rot,
            keep_agent_at_new_pose=not (collided or task.is_stop_called),
        )
        self.update_metrics(collided, backwards)
        return agent_observations

    def update_metrics(self, collided, backwards_motion):
        self._sim._prev_sim_obs["collided"] = collided
        self._sim._prev_sim_obs["backwards_motion"] = backwards_motion

    def teleport(self):
        # Stop is called if both linear/angular speed are below their threshold
        stop = (
            abs(self.linear_velocity) < self.min_abs_lin_speed
            and abs(self.angular_velocity) < self.min_abs_ang_speed
        )
        if stop:
            final_position, final_rotation, backwards, collided = (
                None, None, False, False,  # fmt: skip
            )
        else:
            agent_state = self._sim.get_agent_state()
            agent_magnum_quat = mn.Quaternion(
                agent_state.rotation.imag, agent_state.rotation.real
            )
            current_rigid_state = RigidState(agent_magnum_quat, agent_state.position)
            angular_velocity = np.deg2rad(self.angular_velocity)
            # negative linear velocity is forward
            self.vel_control.linear_velocity = np.array(
                [0.0, 0.0, -self.linear_velocity]
            )
            self.vel_control.angular_velocity = np.array([0.0, angular_velocity, 0.0])
            goal_rigid_state = self.vel_control.integrate_transform(
                self.time_step, current_rigid_state
            )
            final_position, final_rotation, collided = self.get_next_pos_rot(
                goal_rigid_state
            )
            # negative linear velocity is forward
            backwards = self.linear_velocity > 0.0

        return final_position, final_rotation, backwards, collided, stop

    def get_next_pos_rot(self, goal_rigid_state: RigidState):
        """
        Returns Nones if the agent would collide with an object. Otherwise, will adjust
        goal rigid state based on navmesh and return the new position and rotation.
        :param goal_rigid_state:
        :return: (None, None) or (position, rotation)
        """

        allow_sliding = self._sim.config.sim_cfg.allow_sliding
        # snap rigid state to navmesh and set state to object/agent
        if allow_sliding:
            step_fn = self._sim.pathfinder.try_step  # type: ignore
        else:
            step_fn = self._sim.pathfinder.try_step_no_sliding  # type: ignore

        agent_state = self._sim.get_agent_state()
        final_position = step_fn(agent_state.position, goal_rigid_state.translation)
        final_rotation = [
            *goal_rigid_state.rotation.vector,
            goal_rigid_state.rotation.scalar,
        ]

        # Check if a collision occured
        dist_moved_before_filter = (
            goal_rigid_state.translation - agent_state.position
        ).dot()
        dist_moved_after_filter = (final_position - agent_state.position).dot()

        # NB: There are some cases where ||filter_end - end_pos|| > 0 when a
        # collision _didn't_ happen. One such case is going up stairs.  Instead,
        # we check to see if the amount moved after the application of the
        # filter is _less_ than the amount moved before the application of the
        # filter.
        EPS = 1e-5
        collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter
        collided = collided or final_position is None

        return final_position, final_rotation, collided


@registry.register_task_action
class DiscreteVelocityMultiAction(RecursiveSimulatorTaskAction):
    name: str = "discrete_velocity_multi_action"

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)

        self.args = args
        self.kwargs = kwargs

        self.vel_tiling_scheme = self._config.vel_tiling_scheme
        self._lin_vel_tile_density = self._config.lin_vel_tile_density  # units / tile
        self._ang_vel_tile_density = self._config.ang_vel_tile_density  # units / tile
        self._lin_vel_num_tiles = self._config.lin_vel_num_tiles
        self._ang_vel_num_tiles = self._config.ang_vel_num_tiles
        self.min_lin_vel, self.max_lin_vel = self._config.lin_vel_range
        self.min_ang_vel, self.max_ang_vel = self._config.ang_vel_range
        self.use_stop_action = self._config.use_stop_action
        self.time_step = self._config.time_step

        assert self.vel_tiling_scheme in [
            "density",
            "num",
        ], "vel_tiling_scheme must be either 'density' or 'num'"

        self.action_space = self.create_action_space()

    @property
    def action_space(self):
        # Needed to override parent method
        return self._action_space

    @action_space.setter
    def action_space(self, action_space):
        # Needed because of property method
        self._action_space = action_space

    @property
    def lin_vel_tile_density(self):
        if self.vel_tiling_scheme == "density":
            return self._lin_vel_tile_density
        elif self.vel_tiling_scheme == "num":
            return (self.max_lin_vel - self.min_lin_vel) / self._lin_vel_num_tiles

    @property
    def ang_vel_tile_density(self):
        if self.vel_tiling_scheme == "density":
            return self._ang_vel_tile_density
        elif self.vel_tiling_scheme == "num":
            return (self.max_ang_vel - self.min_ang_vel) / self._ang_vel_num_tiles

    @property
    def lin_vel_num_tiles(self):
        if self.vel_tiling_scheme == "density":
            return int(
                (self.max_lin_vel - self.min_lin_vel) / self.lin_vel_tile_density
            )
        elif self.vel_tiling_scheme == "num":
            return self._lin_vel_num_tiles

    @property
    def ang_vel_num_tiles(self):
        if self.vel_tiling_scheme == "density":
            return int(
                (self.max_ang_vel - self.min_ang_vel) / self.ang_vel_tile_density
            )
        elif self.vel_tiling_scheme == "num":
            return self._ang_vel_num_tiles

    def get_lin_vel_from_tile(self, tile):
        return self.min_lin_vel + tile * self.lin_vel_tile_density

    def get_ang_vel_from_tile(self, tile):
        return self.min_ang_vel + tile * self.ang_vel_tile_density

    def create_action_space(self):
        action_space_dict = {}
        for i in range(self.lin_vel_num_tiles):
            for j in range(self.ang_vel_num_tiles):
                key = f"lin{i}_ang{j}"
                linear_velocity = self.get_lin_vel_from_tile(i)
                angular_velocity = self.get_ang_vel_from_tile(j)
                action_space_dict[key] = DiscreteVelocitySingleAction(
                    linear_velocity=linear_velocity,
                    angular_velocity=angular_velocity,
                    *self.args,
                    **self.kwargs,
                )
        action_space_dict["lin0_ang0"] = DiscreteVelocitySingleAction(
            linear_velocity=0,
            angular_velocity=0,
            *self.args,
            **self.kwargs,
        )
        if self.use_stop_action:
            action_space_dict["stop"] = StopAction(*self.args, **self.kwargs)

        return action_space_dict
