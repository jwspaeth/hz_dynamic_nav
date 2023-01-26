import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

from habitat.config.default_structured_configs import SimulatorConfig
from hydra.core.config_store import ConfigStore
from omegaconf import II, MISSING

cs = ConfigStore.instance()


@dataclass
class DynamicNavSimulatorConfig(SimulatorConfig):
    type = "DynamicNav"
    PEOPLE_MASK: bool
    NUM_PEOPLE: int
    PEOPLE_LIN_SPEED: float
    PEOPLE_ANG_SPEED: float
    TIME_STEP: float


cs.store(
    package="habitat.simulator",
    group="hz_dynamic_nav/simulator",
    name="dynamic_nav_sim_config_base",
    node=DynamicNavSimulatorConfig,
)


from hydra.core.config_search_path import ConfigSearchPath
from hydra.core.plugins import Plugins
from hydra.plugins.search_path_plugin import SearchPathPlugin


class HzDynamicNavConfigPlugin(SearchPathPlugin):
    def manipulate_search_path(self, search_path: ConfigSearchPath) -> None:
        search_path.append(
            provider="hz_dynamic_nav",
            path="pkg://hz_dynamic_nav/config/",
        )


Plugins.instance().register(HzDynamicNavConfigPlugin)
