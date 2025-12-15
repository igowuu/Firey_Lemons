from components.drivetrain import DriveTrain
import magicbot

class LemonBot(magicbot.MagicRobot):
    def createObjects(self) -> None:
        self.drivetrain = DriveTrain()

    def teleopPeriodic(self) -> None:
        self.drivetrain.drive_with_joystick()
        self.drivetrain.update_odometry()