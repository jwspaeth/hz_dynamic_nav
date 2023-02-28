from habitat.core.embodied_task import SimulatorTaskAction


class RecursiveSimulatorTaskAction(SimulatorTaskAction):
    """
    This class is a wrapper around a SimulatorTaskAction that triggers recursively extracting the internal actions.
    This is useful for when you want to use a SimulatorTaskAction as a sub-action of another SimulatorTaskAction
    (DiscreteVelocityAction).
    """

    pass
