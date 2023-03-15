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


@hydra.main(
    version_base=None,
    config_path="config",
    config_name="pointnav/ppo_pointnav_example",
)
def main(cfg: "DictConfig"):
    cfg = patch_config(cfg)
    with read_write(cfg):
        OmegaConf.resolve(cfg)
    print(OmegaConf.to_yaml(cfg))
    execute_exp(cfg, "eval" if cfg.habitat_baselines.evaluate else "train")


if __name__ == "__main__":
    register_hydra_plugin(HabitatBaselinesConfigPlugin)
    register_hydra_plugin(HzDynamicNavConfigPlugin)
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
