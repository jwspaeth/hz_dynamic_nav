import functools

from omegaconf import OmegaConf

from . import config, sims, tasks, utils


def print_config_decorator(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        config = func(*args, **kwargs)
        print(OmegaConf.to_yaml(config))
        return config

    return wrapper
