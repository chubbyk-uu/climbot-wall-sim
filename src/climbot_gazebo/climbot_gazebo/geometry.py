"""Quaternion and angle helpers shared by the simulation nodes."""

# Quaternions are (x, y, z, w) tuples, matching the geometry_msgs field order.

import math


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_multiply(first, second):
    """Return the product of two (x, y, z, w) quaternions."""
    first_x, first_y, first_z, first_w = first
    second_x, second_y, second_z, second_w = second
    return (
        first_w * second_x + first_x * second_w + first_y * second_z - first_z * second_y,
        first_w * second_y - first_x * second_z + first_y * second_w + first_z * second_x,
        first_w * second_z + first_x * second_y - first_y * second_x + first_z * second_w,
        first_w * second_w - first_x * second_x - first_y * second_y - first_z * second_z,
    )


def quaternion_conjugate(quaternion):
    """Return the conjugate, which inverts a unit quaternion."""
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def quaternion_from_rpy(roll, pitch, yaw):
    """Return the (x, y, z, w) quaternion for fixed-axis roll, pitch, yaw."""
    cosine_roll = math.cos(roll * 0.5)
    sine_roll = math.sin(roll * 0.5)
    cosine_pitch = math.cos(pitch * 0.5)
    sine_pitch = math.sin(pitch * 0.5)
    cosine_yaw = math.cos(yaw * 0.5)
    sine_yaw = math.sin(yaw * 0.5)
    return (
        sine_roll * cosine_pitch * cosine_yaw - cosine_roll * sine_pitch * sine_yaw,
        cosine_roll * sine_pitch * cosine_yaw + sine_roll * cosine_pitch * sine_yaw,
        cosine_roll * cosine_pitch * sine_yaw - sine_roll * sine_pitch * cosine_yaw,
        cosine_roll * cosine_pitch * cosine_yaw + sine_roll * sine_pitch * sine_yaw,
    )


def rotate_vector(quaternion, vector):
    """Rotate an (x, y, z) vector by an (x, y, z, w) unit quaternion."""
    x, y, z, w = quaternion
    vector_x, vector_y, vector_z = vector
    cross_x = 2.0 * (y * vector_z - z * vector_y)
    cross_y = 2.0 * (z * vector_x - x * vector_z)
    cross_z = 2.0 * (x * vector_y - y * vector_x)
    return (
        vector_x + w * cross_x + y * cross_z - z * cross_y,
        vector_y + w * cross_y + z * cross_x - x * cross_z,
        vector_z + w * cross_z + x * cross_y - y * cross_x,
    )


def yaw_from_quaternion(quaternion):
    """Return the yaw angle of an (x, y, z, w) quaternion."""
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_tuple(message):
    """Return a geometry_msgs Quaternion as an (x, y, z, w) tuple."""
    return (message.x, message.y, message.z, message.w)
