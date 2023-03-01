from typing import TYPE_CHECKING, Any, Dict, Optional, Union, cast

import numpy as np
from gym import spaces
from gym.spaces.box import Box
from habitat.core.registry import registry
from habitat.core.simulator import Sensor, SensorTypes
from habitat.sims.habitat_simulator.habitat_simulator import HabitatSim

TimeObservation = float


@registry.register_sensor
class TimeSensor(Sensor):
    def __init__(
        self, sim: HabitatSim, config: "DictConfig", *args: Any, **kwargs: Any
    ) -> None:
        super().__init__(config, *args, **kwargs)
        self._sim = sim
        self._config = config

    def _get_uuid(self, *args: Any, **kwargs: Any) -> str:
        return "time_sensor"

    def _get_sensor_type(self, *args: Any, **kwargs: Any) -> SensorTypes:
        return SensorTypes.TENSOR

    def _get_observation_space(self, *args: Any, **kwargs: Any) -> Box:
        return spaces.Box(low=0, high=np.inf, shape=(1,), dtype=np.float32)

    def get_observation(self, *args, **kwargs) -> TimeObservation:
        current_time = self._sim.get_world_time()
        return current_time
