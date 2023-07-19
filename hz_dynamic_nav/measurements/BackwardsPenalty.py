from typing import Any

from habitat import EmbodiedTask, Measure, Simulator, registry
from omegaconf import DictConfig


@registry.register_measure
class BackwardsPenalty(Measure):
    """
    Returns a penalty value if the robot has collided.
    """

    cls_uuid: str = "backwards_penalty"

    def __init__(self, sim: Simulator, config: "DictConfig", *args: Any, **kwargs: Any):
        self._sim = sim
        self._config = config
        self._backwards_penalty = config.backwards_penalty
        super().__init__()

    def _get_uuid(self, *args: Any, **kwargs: Any) -> str:
        return self.cls_uuid

    def reset_metric(self, episode, task, *args: Any, **kwargs: Any):
        self._metric = 0
        # self.update_metric(episode=episode, task=task, *args, **kwargs)  # type: ignore

    def update_metric(self, episode, task: EmbodiedTask, *args: Any, **kwargs: Any):
        if self._sim._prev_sim_obs["backwards_motion"]:
            self._metric = -self._backwards_penalty
        else:
            self._metric = 0
