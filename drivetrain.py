#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
import wpimath.units
import swervemodule
import phoenix6.hardware
from config import *

kMaxSpeed = 3.0  # Meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second


class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:

        # Swerve Modules:
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.301, 0.301)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.301, -0.301)
        self.rearLeftLocation = wpimath.geometry.Translation2d(-0.301, 0.301)
        self.rearRightLocation = wpimath.geometry.Translation2d(-0.301, -0.301)
        
        self.frontLeft = swervemodule.SwerveModule(5,6,11,FrontLeftDrivePID,FrontLeftRotatePID)
        self.frontRight = swervemodule.SwerveModule(3,4,10,FrontRightDrivePID,FrontRightRotatePID)
        self.rearLeft = swervemodule.SwerveModule(7,8,12,RearLeftDrivePID,RearLeftRotatePID)
        self.rearRight = swervemodule.SwerveModule(1,2,9,RearRightDrivePID,RearRightRotatePID)

        self.gyro = phoenix6.hardware.Pigeon2(32)

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.rearLeftLocation,
            self.rearRightLocation)

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            wpimath.geometry.Rotation2d(self.gyro.get_yaw().value_as_double),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition()
            )
        )

        # wpimath.geometry.Rotation2d(self.gyro.get_yaw().value_as_double)

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
        GyroOffset: wpimath.geometry.Rotation2d
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(self.gyro.get_yaw().value_as_double)) + GyroOffset
                )
                if fieldRelative
                else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.rearLeft.setDesiredState(swerveModuleStates[2])
        self.rearRight.setDesiredState(swerveModuleStates[3])

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            wpimath.geometry.Rotation2dwpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(self.gyro.get_yaw().value_as_double)),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition()
            )
        )

    def testMode(self, angle: wpimath.geometry.Rotation2d):
        swerveModuleState = wpimath.kinematics.SwerveModuleState(0,angle)

        self.frontLeft.setDesiredState(swerveModuleState)
        self.frontRight.setDesiredState(swerveModuleState)
        self.rearLeft.setDesiredState(swerveModuleState)
        self.rearRight.setDesiredState(swerveModuleState)

    def updatePID(self):
        #self.frontLeft.setDrivePIDGains(FrontLeftDrivePID)
            self.frontLeft.setRotatePIDGains(FrontLeftRotatePID)
            #self.frontRight.setDrivePIDGains(FrontRightDrivePID)
            self.frontRight.setRotatePIDGains(FrontRightRotatePID)

            #self.rearLeft.setDrivePIDGains(RearLeftDrivePID)
            self.rearLeft.setRotatePIDGains(RearLeftRotatePID)
            #self.rearRight.setDrivePIDGains(RearRightDrivePID)
            self.rearRight.setRotatePIDGains(RearRightRotatePID)


    def getError(self):
        return self.frontLeft.rotatePIDController.getPositionError()
    
    def SwerveDashboard(self):
        MadeArray = [
            (self.frontLeft.rotateEncoder.get_position().value_as_double * math.tau), self.frontLeft.getState().speed,
            (self.frontRight.rotateEncoder.get_position().value_as_double * math.tau), self.frontRight.getState().speed,
            (self.rearLeft.rotateEncoder.get_position().value_as_double * math.tau), self.rearLeft.getState().speed,
            (self.rearRight.rotateEncoder.get_position().value_as_double * math.tau), self.rearRight.getState().speed
        ]
        
        return MadeArray
    
    def getGyro(self):
        return (math.radians(self.gyro.getAngle()))
    
    def getPose(self):
        return self.odometry.getPose()