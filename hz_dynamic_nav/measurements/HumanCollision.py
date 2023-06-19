from typing import TYPE_CHECKING, Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
from habitat.core.embodied_task import EmbodiedTask, Measure
from habitat.core.registry import registry
from habitat.sims.habitat_simulator.habitat_simulator import HabitatSim
from omegaconf import DictConfig


@registry.register_measure
class HumanCollision(Measure):
    """
    If the agent collides with a human, the episode is over.
    """

    def __init__(self, sim: HabitatSim, config: DictConfig, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)
        self._sim = sim
        self._config = config

    def _get_uuid(self, *args: Any, **kwargs: Any) -> str:
        return "human_collision"

    def reset_metric(self, episode, *args: Any, **kwargs: Any):
        self._metric = False

    def update_metric(self, episode, task: EmbodiedTask, *args: Any, **kwargs: Any):
        agent_pos = self._sim.get_agent_state().position
        for p in self._sim.people:
            distance = np.sqrt(
                (p.current_position[0] - agent_pos[0]) ** 2
                + (p.current_position[1] - agent_pos[1]) ** 2
                + (p.current_position[2] - agent_pos[2]) ** 2
            )
            if distance < self._config.get("TERMINATION_RADIUS", 0.3):
                self._metric = True
                break

        if self._metric:
            task.is_stop_called = True
