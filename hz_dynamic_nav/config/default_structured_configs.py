from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

from habitat.config.default_structured_configs import (
    ActionConfig,
    MeasurementConfig,
    SimulatorConfig,
)
from hydra.core.config_store import ConfigStore
from hz_dynamic_nav.measurements import CollisionPenalty, SumReward

cs = ConfigStore.instance()


@dataclass
class DynamicNavSimulatorConfig(SimulatorConfig):
    type: str = "DynamicNav"
    people_mask: bool = False
    people_spawn_type: str = "density"
    people_density: float = 0.5
    people_num: int = 3
    people_lin_speed: float = 0.25
    people_ang_speed: float = 10
    time_step: float = 1.0


@dataclass
class VelocityControlActionConfig(ActionConfig):
    type: str = "VelocityAction"  # must be same name as class defined below!!
    # meters/sec:
    lin_vel_range: List[float] = field(default_factory=lambda: [0.0, 0.25])
    # deg/sec:
    ang_vel_range: List[float] = field(default_factory=lambda: [-10.0, 10.0])
    # # meters/sec:
    # lin_vel_range: List[float] = field(default_factory=lambda: [1.0, 1.0])
    # # deg/sec:
    # ang_vel_range: List[float] = field(default_factory=lambda: [1.0, 1.0])
    min_abs_lin_speed: float = 0.025  # meters/sec
    min_abs_ang_speed: float = 1.0  # deg/sec
    time_step: float = 1.0  # seconds


@dataclass
class HumanCollisionMeasurementConfig(MeasurementConfig):
    type: str = "HumanCollision"


@dataclass
class CollisionPenaltyMeasurementConfig(MeasurementConfig):
    type: str = "CollisionPenalty"
    collision_penalty: float = 0.003


@dataclass
class AccCollisionPenaltyMeasurementConfig(MeasurementConfig):
    type: str = "AccCollisionPenalty"
    collision_penalty: float = 0.003


@dataclass
class SumRewardMeasurementConfig(MeasurementConfig):
    type: str = SumReward.__name__
    reward_terms: List[str] = field(
        # available options are "disk" and "tensorboard"
        default_factory=list
    )
    reward_coefficients: List[str] = field(
        # available options are "disk" and "tensorboard"
        default_factory=list
    )


cs.store(
    package="habitat.simulator",
    group="habitat/simulator",
    name="dynamic_nav_sim_config_base",
    node=DynamicNavSimulatorConfig,
)

cs.store(
    package="habitat.task.actions.velocity_control",
    group="habitat/task/actions",
    name="velocity_control",
    node=VelocityControlActionConfig,
)

cs.store(
    package="habitat.task.measurements.human_collision",
    group="habitat/task/measurements",
    name="human_collision",
    node=HumanCollisionMeasurementConfig,
)

cs.store(
    package="habitat.task.measurements.collision_penalty",
    group="habitat/task/measurements",
    name="collision_penalty",
    node=CollisionPenaltyMeasurementConfig,
)

cs.store(
    package="habitat.task.measurements.acc_collision_penalty",
    group="habitat/task/measurements",
    name="acc_collision_penalty",
    node=AccCollisionPenaltyMeasurementConfig,
)

cs.store(
    package=f"habitat.task.measurements.sum_reward",
    group="habitat/task/measurements",
    name="sum_reward",
    node=SumRewardMeasurementConfig,
)

from hydra.core.config_search_path import ConfigSearchPath
from hydra.plugins.search_path_plugin import SearchPathPlugin


class HzDynamicNavConfigPlugin(SearchPathPlugin):
    def manipulate_search_path(self, search_path: ConfigSearchPath) -> None:
        search_path.append(
            provider="hz_dynamic_nav",
            path="pkg://hz_dynamic_nav/config/",
        )
