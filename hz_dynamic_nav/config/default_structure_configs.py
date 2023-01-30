import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

from habitat.config.default_structured_configs import MeasurementConfig, SimulatorConfig
from hydra.core.config_store import ConfigStore
from omegaconf import II, MISSING

cs = ConfigStore.instance()


@dataclass
class DynamicNavSimulatorConfig(SimulatorConfig):
    type: str = "DynamicNav"
    PEOPLE_MASK: bool = False
    NUM_PEOPLE: int = 0
    PEOPLE_LIN_SPEED: float = 0.0
    PEOPLE_ANG_SPEED: float = 0.0
    TIME_STEP: float = 0.0


@dataclass
class DynamicNavSimulatorConfig(SimulatorConfig):
    type: str = "DynamicNav"
    PEOPLE_MASK: bool = False
    NUM_PEOPLE: int = 3
    PEOPLE_LIN_SPEED: float = 0.25
    PEOPLE_ANG_SPEED: float = 10
    TIME_STEP: float = 1.0


@dataclass
class HumanCollisionMeasurementConfig(MeasurementConfig):
    type: str = "HumanCollision"


cs.store(
    package="habitat.simulator",
    group="habitat/simulator",
    name="dynamic_nav_sim_config_base",
    node=DynamicNavSimulatorConfig,
)

cs.store(
    package="habitat.task.measurements.human_collision",
    group="habitat/task/measurements",
    name="human_collision",
    node=HumanCollisionMeasurementConfig,
)


from hydra.core.config_search_path import ConfigSearchPath
from hydra.plugins.search_path_plugin import SearchPathPlugin


class HzDynamicNavConfigPlugin(SearchPathPlugin):
    def manipulate_search_path(self, search_path: ConfigSearchPath) -> None:
        search_path.append(
            provider="hz_dynamic_nav",
            path="pkg://hz_dynamic_nav/config/",
        )
