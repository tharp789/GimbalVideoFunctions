"""
This script defines the camera specs. Edit this one.
"""

class A8MINI:
    MAX_YAW_DEG = 135.0
    MIN_YAW_DEG = -135.0
    MAX_PITCH_DEG = 25.0
    MIN_PITCH_DEG = -90.0
    MAX_ZOOM = 6.0 # all dig
    FOCAL_LENGTH = 21 # mm
    X_RESOLUTION = 1920 # px
    Y_RESOLUTION = 1080 # px
    HORIZONTAL_FOV = 81 # deg
    VERTICAL_FOV = 90 # deg
    F_STOP = 2.8
    SENSOR_SIZE = 1/1.7 # inch, Sony
    X_OFFSET_FROM_GPS = 0.0 # meters
    Y_OFFSET_FROM_GPS = 0.0 # meters
    Z_OFFSET_FROM_GPS = 0.0 # meters
    # Camera intrinsic matrix (K) in pixels
    # assumes centered principal point and no skew
    K = [[FOCAL_LENGTH * X_RESOLUTION / SENSOR_SIZE, 0,                                     X_RESOLUTION/2],
         [0,                                      FOCAL_LENGTH * Y_RESOLUTION / SENSOR_SIZE, Y_RESOLUTION/2],
         [0,                                      0,                                        1]]

class ZR10:
    MAX_YAW_DEG = 135.0
    MIN_YAW_DEG = -135.0
    MAX_PITCH_DEG = 25.0
    MIN_PITCH_DEG = -90.0
    MAX_ZOOM = 30.0 # 10 optical * 3 digital

class ZT6:
    MAX_YAW_DEG = 270.0
    MIN_YAW_DEG = -270.0
    MAX_PITCH_DEG = 25.0
    MIN_PITCH_DEG = -90.0
    MAX_ZOOM = 6.0 # all digital