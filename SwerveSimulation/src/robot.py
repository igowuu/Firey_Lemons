# SIM ONLY - add logic for Gyro, Sensors, and Actuators if needed
# Inverting motors is probably necessary too

import wpilib
import wpilib.drive
from rev import SparkMax

class Config:
    TURN_SPEED = 1 # Decimal 0-1
    HORIZONTAL_SPEED = 1  # Decimal 0-1

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        BRUSHLESS = SparkMax.MotorType.kBrushless
        self.front_left_motor = SparkMax(11, BRUSHLESS)
        self.front_right_motor = SparkMax(12, BRUSHLESS)
        self.back_left_motor = SparkMax(13, BRUSHLESS)
        self.back_right_motor = SparkMax(14, BRUSHLESS)

        self.drive = wpilib.drive.MecanumDrive(
            self.front_left_motor,
            self.back_left_motor,
            self.front_right_motor,
            self.back_right_motor
        )

        self.lstick = wpilib.Joystick(0)
        self.rstick = wpilib.Joystick(1)

        self.gyro = wpilib.AnalogGyro(1)

        self.motor = wpilib.Jaguar(4)

        self.limit1 = wpilib.DigitalInput(1)
        self.limit2 = wpilib.DigitalInput(2)

        self.position = wpilib.AnalogInput(2)

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopPeriodic(self):
        vy = -self.lstick.getY() * Config.HORIZONTAL_SPEED
        vx = self.lstick.getX() * Config.HORIZONTAL_SPEED
        rot_vol = self.rstick.getX() * Config.TURN_SPEED

        self.drive.driveCartesian(vx, vy, rot_vol)
