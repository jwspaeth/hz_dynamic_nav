from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

from habitat.config.default_structured_configs import (
    ActionConfig,
    EnvironmentConfig,
    FogOfWarConfig,
    LabSensorConfig,
    MeasurementConfig,
    SimulatorConfig,
)
from hydra.core.config_store import ConfigStore
from hz_dynamic_nav.measurements import SumReward

cs = ConfigStore.instance()


@dataclass
class DynamicNavSimulatorConfig(SimulatorConfig):
    type: str = "DynamicNav"
    people_mask: bool = False
    people_spawn_type: str = "density"
    people_density: float = 0.25
    people_num: int = 3
    people_lin_speed: float = 0.25
    people_ang_speed: float = 10
    people_stop_distance: float = 0.5
    time_step: float = 1.0
    match_people_speed: bool = False


@dataclass
class TimeSensorConfig(LabSensorConfig):
    type: str = "TimeSensor"


@dataclass
class ContinuousVelocityControlActionConfig(ActionConfig):
    type: str = "ContinuousVelocityAction"  # must be same name as class defined below!!
    # meters/sec:
    lin_vel_range: List[float] = field(default_factory=lambda: [-0.25, 0.25])
    # deg/sec:
    ang_vel_range: List[float] = field(default_factory=lambda: [-10.0, 10.0])
    min_abs_lin_speed: float = 0.025  # meters/sec
    min_abs_ang_speed: float = 1.0  # deg/sec
    time_step: float = 1.0  # seconds


@dataclass
class DiscreteVelocityMultiActionConfig(ActionConfig):
    type: str = (
        "DiscreteVelocityMultiAction"  # must be same name as class defined below!!
    )
    vel_tiling_scheme: str = "num"
    lin_vel_tile_density: float = 0.05  # m / action
    ang_vel_tile_density: float = 2.0  # deg / action
    lin_vel_num_tiles: int = 4
    ang_vel_num_tiles: int = 4
    # meters/sec:
    lin_vel_range: List[float] = field(default_factory=lambda: [-0.25, 0.25])
    # deg/sec:
    ang_vel_range: List[float] = field(default_factory=lambda: [-10.0, 10.0])
    min_abs_lin_speed: float = 0.025  # meters/sec
    min_abs_ang_speed: float = 1.0  # deg/sec
    use_stop_action: bool = False
    use_flip_action: bool = False
    time_step: float = 1.0  # seconds


@dataclass
class BackwardsPenaltyMeasurementConfig(MeasurementConfig):
    type: str = "BackwardsPenalty"
    backwards_penalty: float = 0.003


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


@dataclass
class SocialTopDownMapMeasurementConfig(MeasurementConfig):
    type: str = "SocialTopDownMap"
    max_episode_steps: int = (
        EnvironmentConfig().max_episode_steps
    )  # TODO : Use OmegaConf II()
    map_padding: int = 3
    map_resolution: int = 1024
    draw_source: bool = True
    draw_border: bool = True
    draw_shortest_path: bool = True
    draw_view_points: bool = True
    draw_goal_positions: bool = True
    # axes aligned bounding boxes
    draw_goal_aabbs: bool = True
    fog_of_war: FogOfWarConfig = FogOfWarConfig()


cs.store(
    package="habitat.simulator",
    group="habitat/simulator",
    name="dynamic_nav_sim_config_base",
    node=DynamicNavSimulatorConfig,
)

cs.store(
    package="habitat.task.lab_sensors.time_sensor",
    group="habitat/task/lab_sensors",
    name="time_sensor",
    node=TimeSensorConfig,
)

cs.store(
    package="habitat.task.actions.continuous_velocity_control",
    group="habitat/task/actions",
    name="continuous_velocity_control",
    node=ContinuousVelocityControlActionConfig,
)

cs.store(
    package="habitat.task.actions.discrete_velocity_multi_action",
    group="habitat/task/actions",
    name="discrete_velocity_multi_action",
    node=DiscreteVelocityMultiActionConfig,
)

cs.store(
    package="habitat.task.measurements.backwards_penalty",
    group="habitat/task/measurements",
    name="backwards_penalty",
    node=BackwardsPenaltyMeasurementConfig,
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
cs.store(
    package="habitat.task.measurements.social_top_down_map",
    group="habitat/task/measurements",
    name="social_top_down_map",
    node=SocialTopDownMapMeasurementConfig,
)

from hydra.core.config_search_path import ConfigSearchPath
from hydra.plugins.search_path_plugin import SearchPathPlugin


class HzDynamicNavConfigPlugin(SearchPathPlugin):
    def manipulate_search_path(self, search_path: ConfigSearchPath) -> None:
        search_path.append(
            provider="hz_dynamic_nav",
            path="pkg://hz_dynamic_nav/config/",
        )
