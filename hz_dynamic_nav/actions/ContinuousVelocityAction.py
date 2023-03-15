from typing import TYPE_CHECKING, Any, Dict, List, Optional, Sequence, Tuple, Union

import magnum as mn
import numpy as np
from gym import spaces
from habitat.core.dataset import Episode
from habitat.core.embodied_task import EmbodiedTask, SimulatorTaskAction
from habitat.core.registry import registry
from habitat.core.spaces import ActionSpace
from habitat.tasks.nav.nav import NavigationEpisode, NavigationTask
from habitat_sim import RigidState
from habitat_sim._ext.habitat_sim_bindings import VelocityControl


@registry.register_task_action
class ContinuousVelocityAction(SimulatorTaskAction):
    name: str = "continuous_velocity_control"

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)
        self.vel_control = VelocityControl()
        self.vel_control.controlling_lin_vel = True
        self.vel_control.controlling_ang_vel = True
        self.vel_control.lin_vel_is_local = True
        self.vel_control.ang_vel_is_local = True

        self.min_lin_vel, self.max_lin_vel = self._config.lin_vel_range
        self.min_ang_vel, self.max_ang_vel = self._config.ang_vel_range
        self.min_abs_lin_speed = self._config.min_abs_lin_speed
        self.min_abs_ang_speed = self._config.min_abs_ang_speed
        self.time_step = self._config.time_step

    @property
    def action_space(self):
        return ActionSpace(
            {
                "angular_velocity": spaces.Box(
                    low=np.array([-1.0]), high=np.array([1.0]), dtype=np.float32
                ),
                "linear_velocity": spaces.Box(
                    low=np.array([-1.0]), high=np.array([1.0]), dtype=np.float32
                ),
            }
        )

    def reset(
        self, task: EmbodiedTask, episode: NavigationEpisode, *args: Any, **kwargs: Any
    ):
        task.is_stop_called = False

    def step(
        self,
        *args: Any,
        task: EmbodiedTask,
        angular_velocity: np.ndarray,  # SPELLING MUST MATCH KEY IN self.action_space
        linear_velocity: np.ndarray,  # SPELLING MUST MATCH KEY IN self.action_space
        **kwargs: Any,
    ):
        r"""Moves the agent with a provided linear and angular velocity for the
        provided amount of time
        Args:
            linear_velocity: between [-1,1], scaled according to
                             config.lin_vel_range
            angular_velocity: between [-1,1], scaled according to
                             config.ang_vel_range
        """
        # Extract from single-value array and convert from [-1, 1] to [0, 1] range
        linear_velocity = (linear_velocity[0] + 1.0) / 2.0
        angular_velocity = (angular_velocity[0] + 1.0) / 2.0

        # Scale actions
        linear_velocity = self.min_lin_vel + linear_velocity * (
            self.max_lin_vel - self.min_lin_vel
        )
        angular_velocity = self.min_ang_vel + angular_velocity * (
            self.max_ang_vel - self.min_ang_vel
        )

        final_pos, final_rot, backwards, collided, task.is_stop_called = self.teleport(
            linear_velocity, angular_velocity
        )
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

    def teleport(self, linear_velocity: float, angular_velocity: float):
        # Stop is called if both linear/angular speed are below their threshold
        stop = (
            abs(linear_velocity) < self.min_abs_lin_speed
            and abs(angular_velocity) < self.min_abs_ang_speed
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
            angular_velocity = np.deg2rad(angular_velocity)
            # negative linear velocity is forward
            self.vel_control.linear_velocity = np.array([0.0, 0.0, -linear_velocity])
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
