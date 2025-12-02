class DriveConstants:
    DRIVE_GEAR_RATIO = 6.75
    WHEEL_RADIUS = 0.0508 # meters
    MAX_SPEED = 6.0 # meters / sec
    MAX_ANGULAR_SPEED = 9.42 # rot / sec

class FeedForwardConstants:
    DRIVE_kP = 0.6
    DRIVE_kI = 0.0
    DRIVE_kD = 0.0

    TURN_kP = 3.0
    TURN_kI = 0.0
    TURN_kD = 0.0

    DRIVE_kS = 0.17
    DRIVE_kV = 0.104
    DRIVE_kA = 0.01

    TURN_kS = 0.14
    TURN_kV = 0.375
    TURN_kA = 0.0