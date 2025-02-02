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

from zuuu_hal.mobile_base import MobileBase
from zuuu_hal.utils import sign


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


def ik_vel_old(x: float, y: float, rot: float) -> List[float]:
    """Takes 2 linear speeds and 1 rotational speed (robot's egocentric frame)
    and outputs the PWM to apply to each of the 3 motors in an omni setup.

    Args:
        x (float): x speed (between -1 and 1). Positive "in front" of the robot.
        y (float): y speed (between -1 and 1). Positive "to the left" of the robot.
        rot (float): rotational speed (between -1 and 1). Positive counter-clock wise.

    Returns:
        List[float]: A list of calculated values for the 3 cycles.
    """
    cycle_back = -y + rot
    cycle_right = (-y * np.cos(120 * math.pi / 180)) + (x * np.sin(120 * math.pi / 180)) + rot
    cycle_left = (-y * np.cos(240 * math.pi / 180)) + (x * np.sin(240 * math.pi / 180)) + rot
    return [cycle_back, cycle_right, cycle_left]
