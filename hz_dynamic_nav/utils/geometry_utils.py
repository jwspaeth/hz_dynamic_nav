try:
    import magnum as mn
except ModuleNotFoundError:
    pass
import numpy as np
from habitat.utils.geometry_utils import quaternion_rotate_vector


def get_heading_error(source, target):
    r"""Computes the difference between two headings (radians); can be negative
    or positive.
    """
    diff = target - source
    if diff > np.pi:
        diff -= np.pi * 2
    elif diff < -np.pi:
        diff += np.pi * 2
    return diff


def quat_to_rad(rotation):
    r"""Returns the yaw represented by the rotation np quaternion"""
    heading_vector = quaternion_rotate_vector(rotation.inverse(), np.array([0, 0, -1]))
    phi = np.arctan2(heading_vector[0], -heading_vector[2])

    return phi
