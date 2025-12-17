from subsystems.drivetrain import DriveTrain
from controls.teleop_drive import TeleopDrive
from dashboard import Dashboard

import magicbot

class LemonBot(magicbot.MagicRobot):
    def createObjects(self) -> None:
        self.drivetrain = DriveTrain()
        self.dashboard = Dashboard(self.drivetrain)

    def robotPeriodic(self):
        self.dashboard.execute()

    def teleopInit(self) -> None:
        self.teleop_drive = TeleopDrive(self.drivetrain)

    def teleopPeriodic(self) -> None:
        self.teleop_drive.execute()
        self.drivetrain.update_odometry()