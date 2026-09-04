from dataclasses import dataclass
import numpy as np


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class UAVState:
    x: float
    y: float
    speed: float
    heading: float


def position(state):
    return np.array([state.x, state.y], dtype=float)


def propagate(state, heading_rate, speed_cmd, dt,
              max_heading_rate=np.deg2rad(50.0),
              min_speed=2.0, max_speed=20.0,
              speed_gain=1.2):
    heading_rate = float(np.clip(
        heading_rate, -max_heading_rate, max_heading_rate
    ))

    acceleration = speed_gain * (float(speed_cmd) - state.speed)
    speed = float(np.clip(
        state.speed + acceleration * dt, min_speed, max_speed
    ))

    heading = wrap_angle(state.heading + heading_rate * dt)

    x = state.x + speed * np.cos(heading) * dt
    y = state.y + speed * np.sin(heading) * dt

    return UAVState(x=x, y=y, speed=speed, heading=heading)
