import sys

import geometry_msgs.msg
import rclpy

from rclpy.node import Node
import pygame
import time
import math
import numpy as np
import traceback
import sys
import threading
import queue
from rclpy.qos import ReliabilityPolicy, QoSProfile
from reachy2_sdk import ReachySDK
import numpy.typing as npt
from reachy2_sdk.parts.joints_based_part import JointsBasedPart


import os

# To be able to use pygame in "headless" mode
# set SDL to use the dummy NULL video driver, so it doesn't need a windowing system.
os.environ["SDL_VIDEODRIVER"] = "dummy"

msg = """
This node takes inputs from a controller and publishes them
as Twist messages in SI units. Tested on a SONY Dual shock 4 controller
and an XBOX controller.

Left joy: holonomic translations
Right joy: rotation

L2/L1 : increase/decrease only linear speed (additive) +-0.05m/s
R2/R1 : increase/decrease only angular speed (additive) +-0.2rad/s

CTRL-C  or press CIRCLE on the controller to quit
"""

# PS4 controller:
# Button  0 = X
# Button  1 = O
# Button  2 = Triangle
# Button  3 = Square
# Button  4 = l1
# Button  5 = r1
# Button  6 = l2
# Button  7 = r2
# Button  8 = share
# Button  9 = options
# Button 10 = ps_button
# Button 11 = joy_left
# Button 12 = joy_right

# XBOX controller:
# Button  0 = A
# Button  1 = B
# Button  2 = X
# Button  3 = Y
# Button  4 = LB
# Button  5 = RB
# Button  6 = back
# Button  7 = start
# Button  8 = big central button
# LT and RT are axis (like a joy)

# When using the XBOX controller, most of it is the same,
# except that you must use Start and Back to increase the max speeds.


# These are integer values between 0 and 100
TORQUE_LIMIT=80
SPEED_LIMIT=25

## Bunch of utility functions

def sign(x):
    if x >= 0:
        return 1
    else:
        return -1


def build_pose_matrix(x: float, y: float, z: float) -> npt.NDArray[np.float64]:
    """Build a 4x4 pose matrix for a given position in 3D space, with the effector at a fixed orientation.

    Args:
        x: The x-coordinate of the position.
        y: The y-coordinate of the position.
        z: The z-coordinate of the position.

    Returns:
        A 4x4 NumPy array representing the pose matrix.
    """
    # The effector is always at the same orientation in the world frame
    return np.array(
        [
            [0, 0, -1, x],
            [0, 1, 0, y],
            [1, 0, 0, z],
            [0, 0, 0, 1],
        ]
    )

def set_speed_and_torque_limits(reachy, torque_limit=100, speed_limit=25) -> None:
    """Set back speed and torque limits of all parts to given value."""
    if not reachy.info:
        reachy._logger.warning("Reachy is not connected!")
        return

    for part in reachy.info._enabled_parts.values():
        if issubclass(type(part), JointsBasedPart):
            part.set_speed_limits(speed_limit)
            part.set_torque_limits(torque_limit)
    time.sleep(0.5)

def draw_square(reachy: ReachySDK) -> None:
    """Draw a square path with Reachy's right arm in 3D space.

    This function commands Reachy's right arm to move in a square pattern
    using four predefined positions (A, B, C, and D) in the world frame.
    The square is drawn by moving the arm sequentially through these positions:
    - A: (0.4, -0.5, -0.2)
    - B: (0.4, -0.5, 0)
    - C: (0.4, -0.3, 0)
    - D: (0.4, -0.3, -0.2)

    see https://docs.pollen-robotics.com/sdk/first-moves/kinematics/ for Reachy's coordinate system

    Each movement uses inverse kinematics to calculate the required joint
    positions to achieve the target pose and then sends the commands to
    Reachy's arm to execute the movements.

    Args:
        reachy: An instance of the ReachySDK used to control the robot.
    """
    # Going from A to B
    r_target_pose = build_pose_matrix(0.4, -0.5, 0)
    l_target_pose = build_pose_matrix(0.4, 0.5, 0)
    r_ik = reachy.r_arm.inverse_kinematics(r_target_pose)
    l_ik = reachy.l_arm.inverse_kinematics(l_target_pose)
    reachy.r_arm.goto(r_ik, duration=2.0, degrees=True)
    reachy.l_arm.goto(l_ik, duration=2.0, degrees=True, wait=True)
    
    r_current_pos = reachy.r_arm.forward_kinematics()
    l_current_pos = reachy.l_arm.forward_kinematics()

    # Going from B to C
    r_target_pose = build_pose_matrix(0.4, -0.3, 0)
    l_target_pose = build_pose_matrix(0.4, 0.3, 0)
    r_ik = reachy.r_arm.inverse_kinematics(r_target_pose)
    l_ik = reachy.l_arm.inverse_kinematics(l_target_pose)
    reachy.r_arm.goto(r_ik, duration=2.0, degrees=True)
    reachy.l_arm.goto(l_ik, duration=2.0, degrees=True, wait=True)
    
    r_current_pos = reachy.r_arm.forward_kinematics()
    l_current_pos = reachy.l_arm.forward_kinematics()
    
    # Going from C to D
    r_target_pose = build_pose_matrix(0.4, -0.3, -0.2)
    l_target_pose = build_pose_matrix(0.4, 0.3, -0.2)
    r_ik = reachy.r_arm.inverse_kinematics(r_target_pose)
    l_ik = reachy.l_arm.inverse_kinematics(l_target_pose)
    reachy.r_arm.goto(r_ik, duration=2.0, degrees=True)
    reachy.l_arm.goto(l_ik, duration=2.0, degrees=True, wait=True)
    
    r_current_pos = reachy.r_arm.forward_kinematics()
    l_current_pos = reachy.l_arm.forward_kinematics()
    
    # Going from D to A
    r_target_pose = build_pose_matrix(0.4, -0.5, -0.2)
    l_target_pose = build_pose_matrix(0.4, 0.5, -0.2)
    r_ik = reachy.r_arm.inverse_kinematics(r_target_pose)
    l_ik = reachy.l_arm.inverse_kinematics(l_target_pose)
    reachy.r_arm.goto(r_ik, duration=2.0, degrees=True)
    reachy.l_arm.goto(l_ik, duration=2.0, degrees=True, wait=True)
    
    r_current_pos = reachy.r_arm.forward_kinematics()
    l_current_pos = reachy.l_arm.forward_kinematics()

    

def goto_to_point_A(reachy: ReachySDK) -> None:
    """Move Reachy's right arm to Point A in 3D space.

    This function commands Reachy's right arm to move to a specified target position
    (Point A) in the world frame, which is located at (0.4, -0.5, -0.2).

    Args:
        reachy: An instance of the ReachySDK used to control the robot.
    """
    # position of point A in space
    r_target_pose = build_pose_matrix(0.4, -0.5, -0.2)
    l_target_pose = build_pose_matrix(0.4, 0.5, -0.2)
    # get the position in the joint space
    r_joints_positions = reachy.r_arm.inverse_kinematics(r_target_pose)
    l_joints_positions = reachy.l_arm.inverse_kinematics(l_target_pose)
    # move Reachy's right arm to this point
    reachy.r_arm.goto(r_joints_positions, duration=4, wait=False)
    reachy.l_arm.goto(l_joints_positions, duration=4, wait=True)

def double_square(reachy):
    print("Draw a square with both arms ...")
    goto_to_point_A(reachy)
    draw_square(reachy)
    reachy.goto_posture("default", wait=True)

def open_grippers(reachy):
    print("Opening grippers ...")
    reachy.r_arm.gripper.open()
    reachy.l_arm.gripper.open()

def close_grippers(reachy):
    print("Closing grippers ...")
    reachy.r_arm.gripper.close()
    reachy.l_arm.gripper.close()


## Actual node and controller code

class JoyTeleop(Node):
    def __init__(self):
        super().__init__("zuuu_teleop_joy")
        self.get_logger().info("Starting zuuu_teleop_joy!")

        pygame.init()
        pygame.display.init()
        pygame.joystick.init()

        self.nb_joy = pygame.joystick.get_count()
        if self.nb_joy < 1:
            self.get_logger().error("No controller detected.")
            self.emergency_shutdown()
        self.get_logger().info("nb joysticks: {}".format(self.nb_joy))
        self.j = pygame.joystick.Joystick(0)
        self.lin_speed_ratio = 0.15
        self.rot_speed_ratio = 1.5
        # The joyticks dont come back at a perfect 0 position when released.
        # Any abs(value) below min_joy_position will be assumed to be 0
        self.min_joy_position = 0.03
        self.pub = self.create_publisher(geometry_msgs.msg.Twist, "cmd_vel", 10)
        self.create_timer(0.01, self.main_tick)
        self.current_command = None
        self.command_lock = threading.Lock()
        
        self.emergency_reachy = ReachySDK(host="localhost")

        if not self.emergency_reachy.is_connected:
            exit("Reachy is not connected.")
        self.emergency_reachy.turn_on()
        self.get_logger().info("ReachySDK initialized and turned on.")
        set_speed_and_torque_limits(self.emergency_reachy, torque_limit=TORQUE_LIMIT, speed_limit=SPEED_LIMIT)
        
                
        self.sdk_command_queue = queue.Queue()

        # Start ReachySDK in a separate thread
        self.sdk_thread = threading.Thread(target=self.run_reachy_sdk, daemon=True)
        self.sdk_thread.start()
        
        self.get_logger().info(msg)
        
        
    def set_command(self, command):
        """Set a new command if none is currently being executed."""
        with self.command_lock:
            if self.current_command is None:
                self.current_command = command
            else:
                self.get_logger().warn("Cannot set command; one is already being executed.")

    def clear_command(self):
        """Clear the current command, e.g., during an emergency stop."""
        with self.command_lock:
            self.current_command = None


    def run_reachy_sdk(self):
        """Thread for managing ReachySDK client."""
        try:
            reachy = ReachySDK(host="localhost")

            if not reachy.is_connected:
                exit("Reachy is not connected.")


            while True:
                time.sleep(0.01)  # Avoid busy waiting
                
                with self.command_lock:
                    command = self.current_command
                    self.current_command = None  # Clear the command after reading

                if command is None:
                    continue

                # Execute the command
                try:
                    if command == "double_square":
                        double_square(reachy)
                    elif command == "open_grippers":
                        open_grippers(reachy)
                    elif command == "close_grippers":
                        close_grippers(reachy)
                    else:
                        self.get_logger().error(f"Unknown command: {command}")
                except Exception as e:
                    self.get_logger().error(f"Error executing ReachySDK command: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ReachySDK: {e}")
            
    def emergency_shutdown(self):
        self.get_logger().warn("Emergency shutdown! Spamming a Twist of 0s and turning off Reachy.")
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        # Publishing it once before the turn off 
        self.pub.publish(twist)
        
        self.emergency_reachy.turn_off_smoothly()
        
        while True:    
            self.pub.publish(twist)
            time.sleep(0.01)

    def tick_controller(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.emergency_shutdown()
            elif event.type == pygame.JOYBUTTONDOWN:
                if self.j.get_button(1):
                    self.get_logger().warn("Pressed emergency stop!")
                    self.emergency_shutdown()
                if self.j.get_button(6):  # l2
                    self.lin_speed_ratio = min(3.0, self.lin_speed_ratio + 0.05)
                    self.get_logger().info(
                        "max translational speed: {:.1f}m/s, max rotational speed: {:.1f}rad/s".format(
                            self.lin_speed_ratio * 100, self.rot_speed_ratio * 100
                        )
                    )
                if self.j.get_button(7):  # r2
                    self.rot_speed_ratio = min(12.0, self.rot_speed_ratio + 0.2)
                    self.get_logger().info(
                        "max translational speed: {:.1f}m/s, max rotational speed: {:.1f}rad/s".format(
                            self.lin_speed_ratio * 100, self.rot_speed_ratio * 100
                        )
                    )
                if self.j.get_button(4):  # l1
                    self.lin_speed_ratio = max(0.0, self.lin_speed_ratio - 0.05)
                    self.get_logger().info(
                        "max translational speed: {:.1f}m/s, max rotational speed: {:.1f}rad/s".format(
                            self.lin_speed_ratio * 100, self.rot_speed_ratio * 100
                        )
                    )
                if self.j.get_button(5):  # r1
                    self.rot_speed_ratio = max(0.0, self.rot_speed_ratio - 0.2)
                    self.get_logger().info(
                        "max translational speed: {:.1f}m/s, max rotational speed: {:.1f}rad/s".format(
                            self.lin_speed_ratio * 100, self.rot_speed_ratio * 100
                        )
                    )
                if self.j.get_button(3):  # Y
                    self.set_command("double_square")
                if self.j.get_button(2):  # X
                    self.set_command("open_grippers")
                if self.j.get_button(0):  # A
                    self.set_command("close_grippers")
            elif event.type == pygame.JOYBUTTONUP:
                pass

        if self.nb_joy != pygame.joystick.get_count():
            self.get_logger().warn("Controller disconnected!")
            self.emergency_shutdown()

    def rumble(self, duration):
        self.rumble_start = time.time()
        self.is_rumble = True
        self.rumble_duration = duration
        # Duration doesn't work, have to do it ourselves
        self.j.rumble(1, 1, 1000)

    def print_controller(self):
        # Get the name from the OS for the controller/joystick.
        name = self.j.get_name()
        self.get_logger().info("Joystick name: {}".format(name))

        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = self.j.get_numaxes()
        self.get_logger().info("Number of axes: {}".format(axes))

        for i in range(axes):
            axis = self.j.get_axis(i)
            self.get_logger().info("Axis {} value: {:>6.3f}".format(i, axis))

        buttons = self.j.get_numbuttons()
        self.get_logger().info("Number of buttons: {}".format(buttons))

        for i in range(buttons):
            button = self.j.get_button(i)
            self.get_logger().info("Button {:>2} value: {}".format(i, button))

    def speeds_from_joystick(self):
        cycle_max_t = self.lin_speed_ratio  # 0.2*factor
        cycle_max_r = self.rot_speed_ratio  # 0.1*factor

        if abs(self.j.get_axis(1)) < self.min_joy_position:
            x = 0.0
        else:
            x = -self.j.get_axis(1) * cycle_max_t

        if abs(self.j.get_axis(0)) < self.min_joy_position:
            y = 0.0
        else:
            y = -self.j.get_axis(0) * cycle_max_t

        if abs(self.j.get_axis(3)) < self.min_joy_position:
            rot = 0.0
        else:
            rot = -self.j.get_axis(3) * cycle_max_r

        # Making sure that the xy_speed doesn't go beyond a fixed maximum: (some controllers give (1, 1) when pressed diagonaly instead of (0.5, 0.5)
        xy_speed = math.sqrt(x**2 + y**2)
        max_speed_xy = cycle_max_t
        if xy_speed > max_speed_xy:
            # This formula guarantees that the ratio x/y remains the same, while ensuring the xy_speed is equal to max_speed_xy
            new_x = math.sqrt(max_speed_xy**2 / (1 + (y**2) / (x**2)))
            new_y = new_x * y / x
            # The formula can mess up the signs, fixing them here
            x = sign(x) * new_x / sign(new_x)
            y = sign(y) * new_y / sign(new_y)
        return x, y, rot

    def main_tick(self):
        # self.get_logger().info("Tick!!")
        self.tick_controller()
        x, y, theta = self.speeds_from_joystick()
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = theta
        self.pub.publish(twist)
        # self.get_logger().info(
        #     "\nx_vel: {:.2f}m/s, y_vel: {:.2f}m/s, theta_vel: {:.2f}rad/s.\n"
        #     "Max lin_vel: {:.2f}m/s, max rot_vel: {:.2f}rad/s".format(x, y, theta, self.lin_speed_ratio, self.rot_speed_ratio)
        # )
        # self.print_controller()
        # time.sleep(1.0)


def main():
    rclpy.init()
    node = JoyTeleop()

    try:
        rclpy.spin(node)
    except Exception as e:
        traceback.print_exc()
    finally:
        node.emergency_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



