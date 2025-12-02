import math
import typing

from pyfrc.physics.core import PhysicsInterface
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import SwerveDrive4Kinematics
from constants import DriveConstants

if typing.TYPE_CHECKING:
    from robot import MyRobot

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller
        self.robot = robot
        self.drivetrain = robot.drivetrain

        self.modules = (
            self.drivetrain.frontLeft,
            self.drivetrain.frontRight,
            self.drivetrain.backLeft,
            self.drivetrain.backRight,
        )
        self.gyro_sim = self.drivetrain.gyro.sim_state 

        self.pose = Pose2d()

        self.wheel_rotor_positions = {
            'frontLeft': 0.0,
            'frontRight': 0.0,
            'backLeft': 0.0,
            'backRight': 0.0
        }

        self._wheel_names = ["frontLeft", "frontRight", "backLeft", "backRight"]

        self._meters_per_motor_rev = (
            2 * math.pi * DriveConstants.WHEEL_RADIUS
        ) / DriveConstants.DRIVE_GEAR_RATIO

        self.drivetrain.gyro.reset()

    def update_sim(self, now: float, tm_diff: float) -> None:
        vx = self.robot.xSpeed
        vy = self.robot.ySpeed
        omega = self.robot.rot
        
        # Move robot in field:
        chassis_speeds = ChassisSpeeds(vx, vy, omega)
        self.pose = self.physics_controller.drive(chassis_speeds, tm_diff)

        states = self.drivetrain.kinematics.toSwerveModuleStates(chassis_speeds)
        
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_SPEED)

        for name, module, state in zip(self._wheel_names, self.modules, states):
            wheel_speed = state.speed  # m/s

            delta_revs = (wheel_speed * tm_diff) / self._meters_per_motor_rev
            self.wheel_rotor_positions[name] += delta_revs

            steer_angle = state.angle.radians()
            revs = steer_angle / (2 * math.pi)

            module.steer_motor.sim_state.set_raw_rotor_position(revs)
            module.cancoder.sim_state.set_raw_position(revs)

            module.drive_motor.sim_state.set_raw_rotor_position(
                self.wheel_rotor_positions[name]
            )
        
        yaw_deg = self.pose.rotation().degrees()
        self.gyro_sim.set_raw_yaw(yaw_deg)

        self.drivetrain.updateOdometry()