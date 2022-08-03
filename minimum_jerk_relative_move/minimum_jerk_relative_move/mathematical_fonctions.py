from math import atan2, sin, cos, asin

def euler_from_quaternion(x, y, z, w):
    euler_angle = {"x":0, "y":0, "z":0}
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    euler_angle["x"] = atan2(t0, t1)
    t2 = 2.0 * (w * y - z * x)
    if (t2 > 1.0):
        t2 = 1.0
    elif (t2 < -1.0):
        t2 = -1.0
    euler_angle["y"] = asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    euler_angle["z"] = atan2(t3, t4)
    return euler_angle


def euler_to_quaternion(roll, pitch, yaw):
    sin_roll = sin(roll/2)
    sin_pitch = sin(pitch/2)
    sin_yaw = sin(yaw/2)
    cos_pitch = cos(pitch/2)
    cos_yaw = cos(yaw/2)
    cos_roll = cos(roll/2)
    quaternion = {"x": sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
    "y": cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
    "z": cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,
    "w": cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw}
    return quaternion
