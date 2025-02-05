"""
lidar_safety.py

This module provides a utility class, LidarSafety, to help reduce the robot's speed when
obstacles are detected by the LIDAR. The functional behavior is as follows:
  - safety_distance must be greater than or equal to critical_distance.
  - The robot’s speed is slowed down if the direction of motion matches the direction of at least one LIDAR point within the safety_distance range.
  - The robot’s speed is set to zero if the direction of motion matches that of any LIDAR point within the critical_distance range.
  - If at least one point is in the critical range, even motions moving away from obstacles are slowed to the "safety zone" speed.

Note:
    This module bypasses TF transforms for performance reasons, though a static TF2 transform may be used in the future.
"""

import math
import traceback
from typing import List, Tuple

import cv2
import numpy as np
from reachy_config import ReachyConfig
from sensor_msgs.msg import LaserScan

from zuuu_hal.utils import angle_diff


class LidarSafety:
    def __init__(
        self,
        safety_distance: float,
        critical_distance: float,
        robot_collision_radius: float,
        speed_reduction_factor: float,
        logger,
        fake_hardware: bool = False,
    ) -> None:
        """Utility class to reduce the robot's speed when obstacles are too close as detected by the LIDAR.

        Functional behavior:
          - safety_distance >= critical_distance.
          - The robot's speed is reduced if the direction of motion matches the direction of at least one LIDAR point within the safety_distance range.
          - The robot's speed is set to zero if the direction of motion matches that of any LIDAR point within the critical_distance range.
          - If any point is within the critical distance, even motions moving away from the obstacles are reduced to the safety zone speed.

        Args:
            safety_distance (float): Distance within which obstacles cause speed reduction.
            critical_distance (float): Distance within which obstacles force the robot to stop (for x, y).
            robot_collision_radius (float): The robot's collision radius used to compute forbidden angular spans.
            speed_reduction_factor (float): Factor by which speeds are reduced in unsafe conditions.
            logger: Logger for logging messages.
            fake_hardware (bool, optional): Flag for simulation mode. Defaults to False.
        """
        self.safety_distance = safety_distance
        self.critical_distance = critical_distance
        self.robot_collision_radius = robot_collision_radius
        self.speed_reduction_factor = speed_reduction_factor
        self.fake_hardware = fake_hardware

        # Lists of forbidden angle pairs: [center_angle, half_angle_span]
        self.unsafe_angles: List[List[float]] = []
        self.critical_angles: List[List[float]] = []
        self.at_least_one_critical = False
        self.logger = logger
        self.obstacle_detection_status = "green"

        # Read hardware version from configuration
        reachy_config = ReachyConfig(no_print=True)
        zuuu_version = reachy_config.mobile_base["version_hard"]

        # Not using TF transforms because this is faster
        # TODO use a static TF2 transform instead
        try:
            float_model = float(zuuu_version)
            if float_model < 1.0:
                self.x_offset = 0.155
            else:
                self.x_offset = 0.1815
        except Exception:
            msg = "ZUUU version can't be processed, check that the 'zuuu_model' tag is " "present in the .reachy.yaml file"
            self.logger.error(msg)
            self.logger.error(traceback.format_exc())
            raise RuntimeError(msg)

    def clear_measures(self) -> None:
        """Clears all previous LIDAR measure-derived forbidden angles."""
        self.unsafe_angles = []
        self.critical_angles = []
        self.at_least_one_critical = False

    def process_scan(self, msg: LaserScan) -> None:
        """Processes a LaserScan message to determine which points may pose a safety hazard.

        It clears previous measures, then for each LIDAR point (ignoring infinite ranges) calculates
        its distance and, based on its intensity and proximity, appends a forbidden angle span to either
        unsafe_angles or critical_angles.
        """
        self.clear_measures()
        ranges: List[float] = []
        intensities: List[float] = []
        nb_critical = 0
        angle_min_offset = 0.0
        min_intensity = 0.1
        if self.fake_hardware:
            # In Gazebo the LIDAR always has an intensity of 0.0, disable intensity check
            min_intensity = -1.0
            angle_min_offset = -math.pi

        for i, r in enumerate(msg.ranges):
            if math.isinf(r):
                continue
            angle = msg.angle_min + angle_min_offset + i * msg.angle_increment
            ranges.append(0.0)
            intensities.append(0.0)

            if r < 0.01:
                # Code for "no detection" (e.g. self-collision filter)
                # Adding an unsafe angle to avoid going fast where we're blind.
                self.unsafe_angles.append(self.create_forbidden_angles(angle, 0.25))
                continue

            dist, _, _ = self.dist_to_point(r, angle)
            if dist < self.critical_distance and (msg.intensities[i] > min_intensity):
                self.at_least_one_critical = True
                self.critical_angles.append(self.create_forbidden_angles(angle, dist))
                ranges[-1] = r
                intensities[-1] = msg.intensities[i]
                nb_critical += 1
            elif dist < self.safety_distance and (msg.intensities[i] > min_intensity):
                self.unsafe_angles.append(self.create_forbidden_angles(angle, dist))

    def safety_check_speed_command(self, x_vel: float, y_vel: float, theta_vel: float) -> List[float]:
        """Limits the input speed command based on detected safety hazards.

        Returns the (possibly reduced) speed commands and sets self.obstacle_detection_status as follows:
            - "green": no obstacle detected.
            - "orange": obstacle detected but not in the desired direction.
            - "red": obstacle detected in the desired direction.

        Args:
            x_vel (float): Desired x speed.
            y_vel (float): Desired y speed.
            theta_vel (float): Desired rotational speed.

        Returns:
            List[float]: Limited [x_vel, y_vel, theta_vel] commands.
        """
        if len(self.unsafe_angles) == 0 and len(self.critical_angles) == 0:
            self.obstacle_detection_status = "green"
            return [x_vel, y_vel, theta_vel]
        elif len(self.critical_angles) > 0:
            # Very close to an obstacle.
            if x_vel == 0.0 and y_vel == 0.0:
                self.obstacle_detection_status = "orange"
                return [0.0, 0.0, theta_vel * self.speed_reduction_factor]
            direction = math.atan2(y_vel, x_vel)
            for pair in self.critical_angles:
                if abs(angle_diff(pair[0], direction)) < pair[1]:
                    self.obstacle_detection_status = "red"
                    return [0.0, 0.0, theta_vel * self.speed_reduction_factor]
            self.obstacle_detection_status = "orange"
            return [
                x_vel * self.speed_reduction_factor,
                y_vel * self.speed_reduction_factor,
                theta_vel * self.speed_reduction_factor,
            ]
        else:
            # Moderately close to an obstacle.
            if x_vel == 0.0 and y_vel == 0.0:
                self.obstacle_detection_status = "green"
                return [0.0, 0.0, theta_vel]
            direction = math.atan2(y_vel, x_vel)
            for pair in self.unsafe_angles:
                if abs(angle_diff(pair[0], direction)) < pair[1]:
                    self.obstacle_detection_status = "orange"
                    return [
                        x_vel * self.speed_reduction_factor,
                        y_vel * self.speed_reduction_factor,
                        theta_vel,
                    ]
            self.obstacle_detection_status = "green"
            return [x_vel, y_vel, theta_vel]

    def dist_to_point(self, r: float, angle: float) -> Tuple[float, float, float]:
        """Calculates the distance between a LIDAR point and the robot's center.

        Converts the point from the LIDAR frame to the base_link frame.

        Args:
            r (float): Range reading from the LIDAR.
            angle (float): Angle of the LIDAR reading.

        Returns:
            Tuple[float, float, float]: (distance, x, y) where x and y are the coordinates in the base_link frame.
        """
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        x += self.x_offset
        dist = math.sqrt(x**2 + y**2)
        return dist, x, y

    def create_forbidden_angles(self, angle: float, dist: float) -> List[float]:
        """Creates a pair [angle, half_forbidden_angle_span] representing a dangerous direction.

        Args:
            angle (float): Center angle.
            dist (float): Distance from the robot; used to compute the span.

        Returns:
            List[float]: [angle, beta], where beta is half of the forbidden angle span.
        """
        beta = abs(math.atan2(self.robot_collision_radius, dist))
        return [angle, beta]

    def create_safety_img(self, msg: LaserScan, range_max: float = 3.0) -> np.ndarray:
        """Creates a safety image from a LaserScan message.

        Args:
            msg (LaserScan): The input LaserScan message.
            range_max (float, optional): Maximum range to visualize (default: 3.0 m).

        Returns:
            np.ndarray: An image (BGR) with white pixels marking detected obstacles.

        Note: If msg is None, returns a dummy value.
        """
        if msg is None:
            return 0, 0

        pixel_per_meter = 250
        image_size = int(range_max * pixel_per_meter)
        height = image_size
        width = image_size
        image = np.zeros((height, width, 3), np.uint8)
        center_x = int(round(width / 2))
        center_y = int(round(height / 2))

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if r < 0.01:
                continue  # Skip "no detection" points.
            _, x_m, y_m = self.dist_to_point(r, angle)
            if msg.intensities[i] > 0.1:
                x = int(round(center_x - y_m * pixel_per_meter))
                y = int(round(center_y - x_m * pixel_per_meter))
                if 0 <= x < width and 0 <= y < height:
                    image[y, x] = (255, 255, 255)  # Note: image indexing is (y, x)
        return image
