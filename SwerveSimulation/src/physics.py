import wpilib.simulation
from pyfrc.physics.core import PhysicsInterface

from wpimath.kinematics import (
    MecanumDriveKinematics,
    MecanumDriveWheelSpeeds
)
from wpimath.geometry import Translation2d
from rev import SparkMaxSim
from wpimath.system.plant import DCMotor

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot

class PhysicsConfig:
    ROBOT_HALF_LENGTH = 0.3   # meters
    ROBOT_HALF_WIDTH = 0.3    # meters
    MAX_WHEEL_SPEED = 7.0    # meters per second
    ACTUATOR_SPEED = 3.0      # meters per second
    ACTUATOR_MIN = 0.0        # meters
    ACTUATOR_MAX = 10.0       # meters

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics = physics_controller
        self.robot = robot

        self.fl_sim = SparkMaxSim(robot.front_left_motor,  DCMotor.NEO(1))
        self.fr_sim = SparkMaxSim(robot.front_right_motor, DCMotor.NEO(1))
        self.bl_sim = SparkMaxSim(robot.back_left_motor,   DCMotor.NEO(1))
        self.br_sim = SparkMaxSim(robot.back_right_motor,  DCMotor.NEO(1))

        half_width  = PhysicsConfig.ROBOT_HALF_LENGTH
        half_length = PhysicsConfig.ROBOT_HALF_WIDTH

        self.kinematics = MecanumDriveKinematics(
            Translation2d(+half_length, +half_width), # front-left motor
            Translation2d(+half_length, -half_width), # front-right motor
            Translation2d(-half_length, +half_width), # back-left motor
            Translation2d(-half_length, -half_width), # back-right motor
        )

        self.dio1 = wpilib.simulation.DIOSim(robot.limit1)
        self.dio2 = wpilib.simulation.DIOSim(robot.limit2)
        self.ain  = wpilib.simulation.AnalogInputSim(robot.position)
        self.gyro = wpilib.simulation.AnalogGyroSim(robot.gyro)
        self.motor_pwm = wpilib.simulation.PWMSim(robot.motor.getChannel())

        self.position = 0.0

    def update_sim(self, _: float, dt: float):
        fl = self.fl_sim.getSetpoint()
        fr = self.fr_sim.getSetpoint()
        bl = self.bl_sim.getSetpoint()
        br = self.br_sim.getSetpoint()

        maxWheelSpeed = PhysicsConfig.MAX_WHEEL_SPEED
        wheelSpeeds = MecanumDriveWheelSpeeds(
            fl * maxWheelSpeed,
            fr * maxWheelSpeed,
            bl * maxWheelSpeed,
            br * maxWheelSpeed,
        )

        chassisSpeeds = self.kinematics.toChassisSpeeds(wheelSpeeds)

        self.physics.drive(chassisSpeeds, dt)

        new_angle = self.gyro.getAngle() + chassisSpeeds.omega * dt
        self.gyro.setAngle(new_angle)

        self.position += self.motor_pwm.getSpeed() * dt * PhysicsConfig.ACTUATOR_SPEED

        if self.position <= PhysicsConfig.ACTUATOR_MIN:
            switch1 = True
            switch2 = False
        elif self.position >= PhysicsConfig.ACTUATOR_MAX:
            switch1 = False
            switch2 = True
        else:
            switch1 = False
            switch2 = False

        self.dio1.setValue(switch1)
        self.dio2.setValue(switch2)

        self.ain.setVoltage(self.position)