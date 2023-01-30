from habitat.config.default_structured_configs import register_hydra_plugin
from habitat_baselines.config.default import get_config as _habitat_baselines_get_config
from hz_dynamic_nav.config.default_structure_configs import HzDynamicNavConfigPlugin
from omegaconf import DictConfig, OmegaConf


def get_config(*args, **kwargs) -> DictConfig:
    """
    Returns habitat config object composed of configs from yaml file (config_path) and overrides.
    """
    register_hydra_plugin(HzDynamicNavConfigPlugin)
    cfg = _habitat_baselines_get_config(*args, **kwargs)
    print(OmegaConf.to_yaml(cfg))
    return cfg
