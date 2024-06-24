import math
import os
import traceback
from subprocess import check_output
from typing import List

import cv2
import numpy as np
from reachy_utils.config import ReachyConfig
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
    ) -> None:
        """Utility class to reduce Zuuu's speed when the mobile base is too close to obstacles seen by the LIDAR.
        Functional behaviour:
        - safety_distance >= critical_distance
        - Zuuu's speed is slowed down if the direction of speed matches the direction of at least 1 LIDAR point in
        the safety_distance range
        - Zuuu's speed is changed to 0 if the direction of speed matches the direction of at least 1 LIDAR point in
        the critical_distance range
        - If at least 1 point is in the critical distance, then even motions that move away from the obstacles are
        slowed down to the "safety_zone" speed
        """
        self.safety_distance = safety_distance
        self.critical_distance = critical_distance
        self.robot_collision_radius = robot_collision_radius
        self.speed_reduction_factor = speed_reduction_factor
        # List of [center_angle, angle_width]. e.g. the forbidden angle is center_angle +-angle_width
        self.unsafe_angles = []
        self.critical_angles = []
        self.at_least_one_critical = False
        self.logger = logger
        self.obstacle_detection_status = "green"

        reachy_config = ReachyConfig()
        zuuu_version = reachy_config.mobile_base_config["version_hard"]

        # Not using the TF transforms because this is faster
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
        """Clears all previous measures"""
        self.unsafe_angles = []
        self.critical_angles = []
        self.at_least_one_critical = False

    def process_scan(self, msg: LaserScan) -> None:
        """Takes as input a LaserScan and finds in it the points that could cause a safety
        hazard based on their proximity"""
        self.clear_measures()
        ranges = []
        intensities = []
        nb_critical = 0
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            ranges.append(0.0)
            intensities.append(0.0)
            if r < 0.01:
                # Code value for "no detection". e.g. the lidar filter that filters self collisions
                # Adding an unsafe angle to avoid going fast where we're blind
                self.unsafe_angles.append(self.create_forbidden_angles(angle, 0.25))
                continue
            dist, _, _ = self.dist_to_point(r, angle)
            if dist < self.critical_distance and (msg.intensities[i] > 0.1):
                self.at_least_one_critical = True
                self.critical_angles.append(self.create_forbidden_angles(angle, dist))
                ranges[-1] = r
                intensities[-1] = msg.intensities[i]
                nb_critical += 1
            elif dist < self.safety_distance and (msg.intensities[i] > 0.1):
                self.unsafe_angles.append(self.create_forbidden_angles(angle, dist))

    def safety_check_speed_command(self, x_vel: float, y_vel: float, theta_vel: float) -> List[float]:
        """Limits the input speed command based on the potential safety hazard
        Arguments:
            x_vel {float} -- The desired x speed
            y_vel {float} -- The desired y speed
            theta_vel {float} -- The desired theta speed

        Returns:
            x_vel: float -- The limited x speed command
            y_vel: float -- The limited y speed command
            theta_vel: float -- The limited theta speed command

        The method also sets the self.obstacle_detection_status attribute to "green", "orange" or "red".
            "green" = no obstacle detected,
            "orange" = obstacle detected but not in the direction we want to go,
            "red" = obstacle detected in the direction we want to go.
        """
        if len(self.unsafe_angles) == 0 and len(self.critical_angles) == 0:
            # There are no close obstacles, the speed commands are left untouched
            self.obstacle_detection_status = "green"
            return x_vel, y_vel, theta_vel
        elif len(self.critical_angles) > 0:
            # Zuuu is very close to an obstacle
            if x_vel == 0.0 and y_vel == 0.0:
                # A pure rotation is OK but still slowed down
                self.obstacle_detection_status = "orange"
                return 0.0, 0.0, theta_vel * self.speed_reduction_factor
            direction = math.atan2(y_vel, x_vel)
            for pair in self.critical_angles:
                if abs(angle_diff(pair[0], direction)) < pair[1]:
                    # If the direction matches a critical angle, the speed is 0 in x and y
                    self.obstacle_detection_status = "red"
                    return 0.0, 0.0, theta_vel * self.speed_reduction_factor
            # The direction does not match a critical angle but the speed is still limited
            self.obstacle_detection_status = "orange"
            return (
                x_vel * self.speed_reduction_factor,
                y_vel * self.speed_reduction_factor,
                theta_vel * self.speed_reduction_factor,
            )
        else:
            # Zuuu is moderately close to an obstacle."
            if x_vel == 0.0 and y_vel == 0.0:
                # A pure rotation is OK
                self.obstacle_detection_status = "green"
                return 0.0, 0.0, theta_vel
            direction = math.atan2(y_vel, x_vel)
            for pair in self.unsafe_angles:
                if abs(angle_diff(pair[0], direction)) < pair[1]:
                    # If the direction matches an unsafe angle, the speed is reduced
                    self.obstacle_detection_status = "orange"
                    return (
                        x_vel * self.speed_reduction_factor,
                        y_vel * self.speed_reduction_factor,
                        theta_vel,
                    )
            # The direction does not match an unsafe angle, the speed commands are left untouched
            self.obstacle_detection_status = "green"
            return x_vel, y_vel, theta_vel

    def dist_to_point(self, r: float, angle: float) -> float:
        """Calculates the distance between a LIDAR point and the center of the robot.
        To do this the frame of the point is changed from the LIDAR frame to the base_footprint frame.
        """
        x = r * math.cos(angle)
        y = r * math.sin(angle)

        x = x + self.x_offset
        dist = math.sqrt(x**2 + y**2)
        return dist, x, y

    def create_forbidden_angles(self, angle: float, dist: float) -> List[float]:
        """Creates a pair [angle, half_forbidden_angle_span].
        This represents the direction span that could be dangerous based on a LIDAR input
        """
        # Half of the forbidden angle span
        beta = abs(math.atan2(self.robot_collision_radius, dist))
        return [angle, beta]

    def create_safety_img(self, msg: LaserScan, range_max: float = 3.0) -> None:
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
                # Code value for "no detection". e.g. the lidar filter that filters self collisions
                continue
            _, x_m, y_m = self.dist_to_point(r, angle)
            if msg.intensities[i] > 0.1:
                x = int(round(center_x - y_m * pixel_per_meter))
                y = int(round(center_y - x_m * pixel_per_meter))

                if x >= 0 and x < width and y >= 0 and y < height:
                    image[y, x] = (255, 255, 255)  # y, x as always

        return image
