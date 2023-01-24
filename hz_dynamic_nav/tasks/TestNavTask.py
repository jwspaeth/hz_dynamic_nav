from typing import TYPE_CHECKING, Any, List, Optional, Sequence, Tuple, Union

from habitat.core.registry import registry

PointNav = registry.get_task("Nav-v0")


@registry.register_task(name="TestNav")
class TestNavTask(PointNav):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def step(self, *args, **kwargs):
        print("123")
        return super().step(*args, **kwargs)
