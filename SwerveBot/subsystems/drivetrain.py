from subsystems.swervemodule import SwerveModule
from wpimath.geometry import Translation2d, Pose2d
from phoenix6.hardware import Pigeon2
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, ChassisSpeeds

from constants.hardware import MAX_SPEED_MPS

class DriveTrain:
    def __init__(self) -> None:
        self.module_locations = (
            Translation2d(0.381, 0.381),    # front left
            Translation2d(0.381, -0.381),   # front right
            Translation2d(-0.381, 0.381),   # back left
            Translation2d(-0.381, -0.381)   # back right
        )

        self.modules = (
            SwerveModule(21, 22, 23),  # front left
            SwerveModule(31, 32, 33),  # front right
            SwerveModule(11, 12, 13),  # back left
            SwerveModule(41, 42, 43)   # back right
        )
        
        self.gyro = Pigeon2(30, canbus="can0")
        self.gyro.reset()

        self.kinematics = SwerveDrive4Kinematics(*self.module_locations)
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.modules[0].get_position(),
                self.modules[1].get_position(),
                self.modules[2].get_position(),
                self.modules[3].get_position()
            )
        )

        self._xSpeed = 0
        self._ySpeed = 0
        self._omega = 0

        self.field_oriented = True

    def get_commanded_chassis_speeds(self) -> ChassisSpeeds:
        if self.field_oriented:
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                self._xSpeed,
                self._ySpeed,
                self._omega,
                self.gyro.getRotation2d(),
            )
        else:
            return ChassisSpeeds(
                self._xSpeed,
                self._ySpeed,
                self._omega,
            )
    
    def get_pose(self) -> Pose2d:
        return self.odometry.getPose()

    def drive(self, xSpeed: float, ySpeed: float, omega: float) -> None:
        """Drive the bot given desired motor speeds."""
        self._xSpeed = xSpeed
        self._ySpeed = ySpeed
        self._omega = omega

        if self.field_oriented:
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, omega, self.gyro.getRotation2d()
            )
        else:
            chassisSpeeds = ChassisSpeeds(xSpeed, ySpeed, omega)

        module_states = self.kinematics.toSwerveModuleStates(chassisSpeeds)
        self.kinematics.desaturateWheelSpeeds(module_states, MAX_SPEED_MPS)
        
        for module, state in zip(self.modules, module_states):
            module.set_desired_state(state)

    def update_odometry(self) -> None:
        """Update bot position relative to field."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.modules[0].get_position(),
                self.modules[1].get_position(),
                self.modules[2].get_position(),
                self.modules[3].get_position(),
            )
        )
    
    def reset_odometry(self, pose: Pose2d = Pose2d()) -> None:
        """Reset bot position."""
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            (
                self.modules[0].get_position(),
                self.modules[1].get_position(),
                self.modules[2].get_position(),
                self.modules[3].get_position(),
            ),
            pose
        )