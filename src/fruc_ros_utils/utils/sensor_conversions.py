#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: MIT License (open source, free to modify and redistribute)
# Repository: fruc_ros_utils
#
# Description:
#   Sensor convention conversions (e.g., IMU NED→ENU).

"""Sensor-frame conversion helpers for ROS messages."""

from sensor_msgs.msg import Imu


def imu_ned_to_enu(imu: Imu) -> Imu:
    """
    Convert IMU orientation, angular velocity, and linear acceleration
    from NED (North-East-Down) to ENU (East-North-Up).

    Args:
        imu: sensor_msgs/Imu message in NED convention.

    Returns:
        sensor_msgs/Imu in ENU convention.
    """
    enu = Imu()
    enu.header = imu.header

    # Angular velocity
    enu.angular_velocity.x = imu.angular_velocity.y
    enu.angular_velocity.y = imu.angular_velocity.x
    enu.angular_velocity.z = -imu.angular_velocity.z

    # Linear acceleration
    enu.linear_acceleration.x = imu.linear_acceleration.y
    enu.linear_acceleration.y = imu.linear_acceleration.x
    enu.linear_acceleration.z = -imu.linear_acceleration.z

    # Orientation (quaternion)
    enu.orientation.x = imu.orientation.y
    enu.orientation.y = imu.orientation.x
    enu.orientation.z = -imu.orientation.z
    enu.orientation.w = imu.orientation.w

    return enu
