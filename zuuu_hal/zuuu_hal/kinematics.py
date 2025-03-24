"""
kinematics.py

This module provides pure functions for computing the kinematics of an omnidirectional 
mobile robot. It includes functions for converting wheel rotational speeds to PWM commands, 
mapping desired robot velocities (linear and angular) to individual wheel speeds, and computing 
odometry from wheel speeds.

See Chapters 13.2 "Omnidirectional Wheeled Mobile Robots" and 13.4 "Odometry" in Modern Robotics 
to understand the formulas used here. 
Note that our robot frame 
differs from the standard conventions as follows (theirs-> ours):
xb -> y, yb -> -x, theta -> -theta, u1 -> uB, u2 -> uL, u3 -> uR.
    
Some functions require a MobileBase instance (from mobile_base.py) to supply hardware constants 
(wheel_radius, wheel_to_center), making them suitable for both physical 
and simulated robots.
"""

import math
from typing import List, Tuple

import numpy as np

# from .mobile_base import MobileBase
# from .utils import sign

from collections import deque
from typing import Any, Deque, Optional

from pyvesc.VESC import MultiVESC


def sign(x: float) -> int:
    """Returns 1 if x >= 0, -1 otherwise"""
    if x >= 0:
        return 1
    else:
        return -1

class MobileBase:
    """Mobile base representation and the interface with low level controllers."""

    def __init__(
        self,
        serial_port: str = "/dev/vesc_wheels",
        left_wheel_id: Optional[int] = 24,
        right_wheel_id: Optional[int] = 72,
        back_wheel_id: Optional[int] = 116,
        fake_hardware: bool = False,
    ) -> None:
        params = [
            {"can_id": left_wheel_id, "has_sensor": True, "start_heartbeat": True},
            {"can_id": right_wheel_id, "has_sensor": True, "start_heartbeat": True},
            {"can_id": back_wheel_id, "has_sensor": True, "start_heartbeat": True},
        ]
        self.fake_hardware: bool = fake_hardware

        # Battery parameters. These values are conservative since the battery cells are nearly empty around 3.3V.
        # The current battery's BMS shuts down at 20V ±1V, so each cell would be ~2.86V ±0.14V.
        self.battery_cell_warn_voltage: float = 3.5
        self.battery_cell_min_voltage: float = 3.3
        self.battery_nb_cells: int = 7
        self.battery_check_period: int = 60

        # Wheel measurement counters and physical parameters.
        self.left_wheel_nones: int = 0
        self.right_wheel_nones: int = 0
        self.back_wheel_nones: int = 0
        self.wheel_radius: float = 0.21 / 2.0
        self.wheel_to_center: float = 0.19588
        self.half_poles: float = 15.0

        # Wheel RPM values.
        self.left_wheel_rpm: float = 0.0
        self.right_wheel_rpm: float = 0.0
        self.back_wheel_rpm: float = 0.0
        self.left_wheel_avg_rpm: float = 0.0
        self.right_wheel_avg_rpm: float = 0.0
        self.back_wheel_avg_rpm: float = 0.0

        # Deques to hold recent RPM measurements.
        self.left_wheel_rpm_deque: Deque[float] = deque(maxlen=10)
        self.right_wheel_rpm_deque: Deque[float] = deque(maxlen=10)
        self.back_wheel_rpm_deque: Deque[float] = deque(maxlen=10)

        # Initialize measurements depending on whether hardware is fake.
        init_measurements_value: Optional[str] = "No measurements in fake_hardware mode" if fake_hardware else None
        self.left_wheel_measurements: Optional[Any] = init_measurements_value
        self.right_wheel_measurements: Optional[Any] = init_measurements_value
        self.back_wheel_measurements: Optional[Any] = init_measurements_value

        if not self.fake_hardware:
            self._multi_vesc = MultiVESC(serial_port=serial_port, vescs_params=params)
            (
                self.left_wheel,
                self.right_wheel,
                self.back_wheel,
            ) = self._multi_vesc.controllers

    def read_all_measurements(self) -> None:
        """Reads all the measurements for the left, right, and back wheels."""
        if self.fake_hardware:
            return
        self.left_wheel_measurements = self.left_wheel.get_measurements()
        self.right_wheel_measurements = self.right_wheel.get_measurements()
        self.back_wheel_measurements = self.back_wheel.get_measurements()

    def deque_to_avg(self, dq: Deque[float]) -> float:
        """Returns the average of the values contained in the deque."""
        if not dq:
            raise ValueError("Deque is empty; cannot compute average.")
        total = 0.0
        for value in dq:
            total += value
        return total / len(dq)

def ik_vel(x_vel: float, y_vel: float, rot_vel: float, mobile_base: MobileBase) -> List[float]:
    """Takes 2 linear speeds and 1 rotational speed (robot's egocentric frame) and outputs the rotational speed (rad/s)
    of each of the 3 motors in an omni setup.

    Args:
        x_vel (float): x speed (m/s). Positive "in front" of the robot.
        y_vel (float): y speed (m/s). Positive "to the left" of the robot.
        rot_vel (float): rotational speed (rad/s). Positive counter-clock wise.
        mobile_base (MobileBase): An instance providing wheel_radius, wheel_to_center, etc.

    Returns:
        List[float]: A list with three wheel speeds (rad/s).
    """
    wheel_rot_speed_back = (1 / mobile_base.wheel_radius) * (mobile_base.wheel_to_center * rot_vel - y_vel)
    wheel_rot_speed_right = (1 / mobile_base.wheel_radius) * (
        mobile_base.wheel_to_center * rot_vel + y_vel / 2.0 + math.sin(math.pi / 3) * x_vel
    )
    wheel_rot_speed_left = (1 / mobile_base.wheel_radius) * (
        mobile_base.wheel_to_center * rot_vel + math.sin(math.pi / 3) * y_vel / 2 - math.sin(math.pi / 3) * x_vel
    )
    return [wheel_rot_speed_back, wheel_rot_speed_right, wheel_rot_speed_left]


def dk_vel(rot_l: float, rot_r: float, rot_b: float, mobile_base: MobileBase) -> Tuple[float, float, float]:
    """Takes the 3 rotational speeds (in rpm) of the 3 wheels and outputs the x linear speed (m/s),
    y linear speed (m/s) and rotational speed (rad/s) in the robot egocentric frame.

    Args:
        rot_l (float): rpm speed of the left wheel.
        rot_r (float): rpm speed of the right wheel.
        rot_b (float): rpm speed of the back wheel.
        mobile_base (MobileBase): Provides wheel_radius, wheel_to_center, etc.

    Returns:
        Tuple[float, float, float]: (x_vel, y_vel, theta_vel).
    """
    # Convert rpm to rad/s then to m/s.
    speed_l = (2 * math.pi * rot_l / 60) * mobile_base.wheel_radius
    speed_r = (2 * math.pi * rot_r / 60) * mobile_base.wheel_radius
    speed_b = (2 * math.pi * rot_b / 60) * mobile_base.wheel_radius

    x_vel = -speed_l * (1 / (2 * math.sin(math.pi / 3))) + speed_r * (1 / (2 * math.sin(math.pi / 3)))
    y_vel = -speed_b * 2 / 3.0 + speed_l * 1 / 3.0 + speed_r * 1 / 3.0
    theta_vel = (speed_l + speed_r + speed_b) / (3 * mobile_base.wheel_to_center)
    return x_vel, y_vel, theta_vel


def wheel_rot_speed_to_pwm_no_friction(rot: float) -> float:
    """Uses a simple linear model to map the expected rotational speed of the wheel to a constant PWM
    (based on measures made on a full Reachy Mobile)
    """
    return rot / 22.7


def wheel_rot_speed_to_pwm(rot: float) -> float:
    """Uses a simple affine model to map the expected rotational speed of the wheel to a constant PWM
    (based on measures made on a full Reachy Mobile)
    """
    # Creating an arteficial null zone to avoid undesired behaviours for very small rot speeds
    epsilon = 0.02
    if rot > epsilon:
        pwm = 0.0418 * rot + 0.0126
    elif rot < -epsilon:
        pwm = 0.0418 * rot - 0.0126
    else:
        pwm = 0.0
    return pwm


def pwm_to_wheel_rot_speed(pwm: float) -> float:
    """Uses a simple affine model to map a PWM to the expected rotational speed of the wheel
    (based on measures made on a full Reachy Mobile)
    """
    # Creating an arteficial null zone to avoid undesired behaviours for very small rot speeds
    if abs(pwm) < 0.0126:
        rot = 0.0
    else:
        rot = sign(pwm) * (abs(pwm) - 0.0126) / 0.0418
    return rot


def ik_vel_to_pwm(x_vel: float, y_vel: float, rot_vel: float, mobile_base: MobileBase) -> List[float]:
    """Takes 2 linear speeds and 1 rotational speed (robot's egocentric frame)
    and outputs the PWM to apply to each of the 3 motors in an omni setup.

    Args:
        x_vel (float): x speed (m/s). Positive "in front" of the robot.
        y_vel (float): y speed (m/s). Positive "to the left" of the robot.
        rot_vel (float): rotational speed (rad/s). Positive counter-clock wise.
        mobile_base (MobileBase): Provides necessary hardware constants.

    Returns:
        List[float]: A list of PWM values for the wheels.
    """
    rot_vels = ik_vel(x_vel, y_vel, rot_vel, mobile_base)
    return [wheel_rot_speed_to_pwm(rot) for rot in rot_vels]


def ik_vel_old(x: float, y: float, rot: float, mobile_base: MobileBase) -> List[float]:
    """Takes 2 linear speeds and 1 rotational speed (robot's egocentric frame)
    and outputs the PWM to apply to each of the 3 motors in an omni setup.

    Args:
        x (float): x speed (between -1 and 1). Positive "in front" of the robot.
        y (float): y speed (between -1 and 1). Positive "to the left" of the robot.
        rot (float): rotational speed (between -1 and 1). Positive counter-clock wise.

    Returns:
        List[float]: A list of calculated values for the 3 cycles.
    """
    rot = rot * 1.8655238095238096
    x = x / 0.10499999999999998
    y = y / 0.10499999999999998
    cycle_back = -y + rot
    cycle_right = (-y * np.cos(120 * math.pi / 180)) + (x * np.sin(120 * math.pi / 180)) + rot
    cycle_left = (-y * np.cos(240 * math.pi / 180)) + (x * np.sin(240 * math.pi / 180)) + rot
    return [cycle_back, cycle_right, cycle_left]


def debug_ik():

    omnibase = MobileBase(
            left_wheel_id=24, right_wheel_id=None, back_wheel_id=116, fake_hardware=True
        )
    x = -0.5
    y = 1.0
    rot = 50.0

    # back, right, left = ik_vel(x, y, rot, omnibase)
    back, right, left = ik_vel_old(x, y, rot)

    print(f"Output: back={back}, right={right}, left={left}")

    x_vel, y_vel, theta_vel = dk_vel(left*60  / (2 * math.pi) , right*60 / (2 * math.pi) , back*60 / (2 * math.pi), omnibase)
    print(f"Input: x_vel={x_vel}, y_vel={y_vel}, theta_vel={theta_vel}")


if __name__ == "__main__":
    debug_ik()
