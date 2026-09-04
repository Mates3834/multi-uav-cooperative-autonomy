import numpy as np


def rotate_offset(offset_xy, heading):
    c = np.cos(heading)
    s = np.sin(heading)
    R = np.array([[c, -s], [s, c]])
    return R @ np.asarray(offset_xy, dtype=float)


def formation_reference(leader_position, leader_heading, body_offset):
    """
    Desired follower location from a body-frame offset relative to the leader.
    """
    return (
        np.asarray(leader_position, dtype=float)
        + rotate_offset(body_offset, leader_heading)
    )


def formation_error(actual_position, desired_position):
    return (
        np.asarray(desired_position, dtype=float)
        - np.asarray(actual_position, dtype=float)
    )
