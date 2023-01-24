from habitat.core.registry import registry
from habitat_baselines.run import main

TestNav = registry.get_task("TestNav")
if __name__ == "__main__":
    main()
