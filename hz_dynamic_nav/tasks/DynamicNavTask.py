import copy
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
from habitat.core.dataset import Episode
from habitat.core.registry import registry
from habitat.core.spaces import ActionSpace, Space
from habitat.tasks.nav.nav import NavigationEpisode, NavigationTask
from hz_dynamic_nav.actions import RecursiveSimulatorTaskAction


@registry.register_task(name="DynamicNav")
class DynamicNavTask(NavigationTask):
    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)

        # Unpack recursive actions
        for key, value in self.actions.items():
            if isinstance(value, RecursiveSimulatorTaskAction):
                self.actions.pop(key, None)
                for k, v in value.action_space.items():
                    self.actions[k] = v

        self._action_keys = list(self.actions.keys())
        self._is_stop_called = False

    def reset(self, episode: Episode):
        self._sim.reset_people()
        episode.people_paths = [p.waypoints for p in self._sim.people]
        observations = super().reset(episode)
        return observations

    def step(self, action: Dict[str, Any], episode: Episode):
        if "action_args" not in action or action["action_args"] is None:
            action["action_args"] = {}
        action_name = action["action"]
        if isinstance(action_name, (int, np.integer)):
            action_name = self.get_action_name(action_name)
        assert (
            action_name in self.actions
        ), f"Can't find '{action_name}' action in {self.actions.keys()}."

        task_action = self.actions[action_name]

        # Move people
        for p in self._sim.people:
            p.step()

        observations = task_action.step(**action["action_args"], task=self)
        observations.update(
            self.sensor_suite.get_observations(
                observations=observations,
                episode=episode,
                action=action,
                task=self,
            )
        )

        self._is_episode_active = self._check_episode_is_active(
            observations=observations, action=action, episode=episode, task=self
        )

        return observations

    @property
    def is_stop_called(self):
        return self._is_stop_called

    @is_stop_called.setter
    def is_stop_called(self, value):
        self._is_stop_called = value

    def _check_episode_is_active(self, *args: Any, **kwargs: Any) -> bool:
        # Manually call these, because we need it to be updated earlier
        # than it is currently updated in env.py.
        self.measurements.measures["distance_to_goal"].update_metric(*args, **kwargs)
        self.measurements.measures["human_collision"].update_metric(*args, **kwargs)

        # End episode if success is True
        success = self.measurements.measures["success"].assess_success_distance(
            *args, **kwargs
        )
        if success == 1.0:
            self.is_stop_called = True
            return False

        result = super()._check_episode_is_active(*args, **kwargs)
        return result

    @property
    def action_space(self) -> Space:
        """
        Hack to fix embodied_task without editing it directly
        :return:
        """

        return ActionSpace(
            {
                action_name: action_instance.action_space
                for action_name, action_instance in self.actions.items()
            }
        )
