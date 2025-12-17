import wpilib
import wpimath.filter

from subsystems.drivetrain import DriveTrain
from constants.hardware import MAX_OMEGA_PS, MAX_SPEED_MPS

class TeleopDrive:
    def __init__(self, drivetrain: DriveTrain):
        self.drivetrain = drivetrain
        self.lstick = wpilib.Joystick(0)
        self.rstick = wpilib.Joystick(1)

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.omegaLimiter = wpimath.filter.SlewRateLimiter(3)

    def execute(self) -> None:
        if self.lstick.getRawButtonPressed(1):
            self.drivetrain.reset_odometry()

        if self.lstick.getRawButtonPressed(2):
            self.field_oriented = not self.field_oriented

        self.xSpeed = (
            self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.lstick.getX(), 0.02)
            ) * MAX_SPEED_MPS
        )

        self.ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.lstick.getY(), 0.02)
            ) * MAX_SPEED_MPS
        )

        self.omega = (
            -self.omegaLimiter.calculate(
                wpimath.applyDeadband(self.rstick.getX(), 0.02)
            ) * MAX_OMEGA_PS
        )

        self.drivetrain.drive(self.xSpeed, self.ySpeed, self.omega)