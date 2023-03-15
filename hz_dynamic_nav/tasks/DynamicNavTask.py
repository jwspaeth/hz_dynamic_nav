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
            observations=observations, action=action, episode=episode
        )

        return observations

    def _check_episode_is_active(self, *args: Any, **kwargs: Any) -> bool:
        result = super()._check_episode_is_active(*args, **kwargs)
        # if not result:
        #     print("Measures: ")
        #     success = self.measurements.measures["success"].get_metric()
        #     distance_to_goal = self.measurements.measures[
        #         "distance_to_goal"
        #     ].get_metric()
        #     human_collision = self.measurements.measures["human_collision"].get_metric()
        #     collisions = self.measurements.measures["collisions"].get_metric()
        #     print(f"\tSuccess: {success}")
        #     print(f"\tDistance to goal: {distance_to_goal}")
        #     print(f"\tHuman collision: {human_collision}")
        #     print(f"\tCollisions: {collisions}")
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
