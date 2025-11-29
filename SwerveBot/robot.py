import wpilib
import wpimath
import wpimath.filter
import drivetrain

class MyRobot(wpilib.TimedRobot):
    """Main bot method"""
    def robotInit(self) -> None:
        self.controller = wpilib.XboxController(0)

        self.drivetrain = drivetrain.Drivetrain()

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(False)
        self.drivetrain.updateOdometry()

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick(True)
        self.drivetrain.updateOdometry()

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), 0.02)
            )
            * drivetrain.MAX_SPEED
        )

        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), 0.02)
            )
            * drivetrain.MAX_SPEED
        )

        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.02)
            )
            * drivetrain.MAX_ANGULAR_SPEED
        )
        
        self.drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())