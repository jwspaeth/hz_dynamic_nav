import sys

import hydra
from habitat.config.default import patch_config
from habitat.config.default_structured_configs import register_hydra_plugin
from habitat_baselines.config.default_structured_configs import (
    HabitatBaselinesConfigPlugin,
)
from habitat_baselines.run import execute_exp
from omegaconf import OmegaConf, read_write

from hz_dynamic_nav.config.default_structured_configs import HzDynamicNavConfigPlugin


def match_people_speed(cfg):
    # Take max speed from action space and assign it to people speed
    # This will only work for velocity action spaces and fail for others
    with read_write(cfg):
        action_keys = list(cfg.habitat.task.actions.keys())
        action_key = action_keys[0]
        cfg.habitat.simulator.people_lin_speed = cfg.habitat.task.actions[
            action_key
        ].lin_vel_range[1]
        cfg.habitat.simulator.people_ang_speed = cfg.habitat.task.actions[
            action_key
        ].ang_vel_range[1]


@hydra.main(
    version_base=None,
    config_path="config",
    config_name="pointnav/ppo_pointnav_example",
)
def main(cfg: "DictConfig"):
    cfg = patch_config(cfg)

    # Resolve config so it can be pickled to all environments
    with read_write(cfg):
        OmegaConf.resolve(cfg)

    # Match people speed to agent speed if flag is set
    if cfg.habitat.simulator.match_people_speed:
        match_people_speed(cfg)

    # Print config for readability
    print(OmegaConf.to_yaml(cfg))
    execute_exp(cfg, "eval" if cfg.habitat_baselines.evaluate else "train")


if __name__ == "__main__":
    # Setup hydra plugins and resolvers
    register_hydra_plugin(HabitatBaselinesConfigPlugin)
    register_hydra_plugin(HzDynamicNavConfigPlugin)

    # Check for old API from habitat_baselines
    if "--exp-config" in sys.argv or "--run-type" in sys.argv:
        raise ValueError(
            "The API of run.py has changed to be compatible with hydra.\n"
            "--exp-config is now --config-name and is a config path inside habitat-baselines/habitat_baselines/config/. \n"
            "--run-type train is replaced with habitat_baselines.evaluate=False (default) and --run-type eval is replaced with habitat_baselines.evaluate=True.\n"
            "instead of calling:\n\n"
            "python habitat-baselines/habitat_baselines/run.py --exp-config habitat-baselines/habitat_baselines/config/<path-to-config> --run-type train/eval\n\n"
            "You now need to do:\n\n"
            "python habitat-baselines/habitat_baselines/run.py --config-name=<path-to-config> habitat_baselines.evaluate=False/True\n"
        )
    main()
