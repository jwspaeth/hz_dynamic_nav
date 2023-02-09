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
    num_people: int = 3
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
class CollisionsMeasurementConfig(MeasurementConfig):
    type: str = "Collisions"


@dataclass
class HumanCollisionMeasurementConfig(MeasurementConfig):
    type: str = "HumanCollision"


@dataclass
class CollisionPenaltyMeasurementConfig(MeasurementConfig):
    type: str = CollisionPenalty.__name__
    collision_penalty: float = 0.003
    # collision_penalty: float = 1.0


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
    name="kinematic_velocity_control",
    node=VelocityControlActionConfig,
)

cs.store(
    package="habitat.task.measurements.collisions",
    group="habitat/task/measurements",
    name="collisions",
    node=CollisionsMeasurementConfig,
)

cs.store(
    package="habitat.task.measurements.human_collision",
    group="habitat/task/measurements",
    name="human_collision",
    node=HumanCollisionMeasurementConfig,
)

cs.store(
    package=f"habitat.task.measurements.{CollisionPenalty.cls_uuid}",
    group="habitat/task/measurements",
    name=f"{CollisionPenalty.cls_uuid}",
    node=CollisionPenaltyMeasurementConfig,
)

cs.store(
    package=f"habitat.task.measurements.{SumReward.cls_uuid}",
    group="habitat/task/measurements",
    name=f"{SumReward.cls_uuid}",
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
