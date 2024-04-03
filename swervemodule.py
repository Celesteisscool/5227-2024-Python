# Code for creating and modifying the pair of motors that make up a swerve drive

import math
#import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import rev
import phoenix6

kWheelRadius = 0.0508
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau

class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        rotateMotorChannel: int,        
        rotateEncoderChannel: int,
        drivePID: tuple,
        rotatePID: tuple ) -> None:
        
        """Constructs a SwerveModule with a drive motor, rotate motor, rotate encoder.  Then set the PID Loop gains
        :param driveMotorAddress:   CAN output for the drive motor drive
        :param rotateMotorAddress:  CAN output for the rotate motor drive
        :param rotateEncoderAddress:CAN input for the rotate encoder 
        :param drivePID:            3-Values for P-I-D to run the drive motor
        :param rotatePID:           3-Values for P-I-D to run the rotate (turning) motor
        """
        # Setup the swerve drive as a new instance in the robot class
        self.driveMotor = rev.CANSparkMax(driveMotorChannel, rev.CANSparkMax.MotorType.kBrushless)
        self.rotateMotor = rev.CANSparkMax(rotateMotorChannel, rev.CANSparkMax.MotorType.kBrushless)
        self.rotateEncoder = phoenix6.hardware.CANcoder(rotateEncoderChannel)

        # Gains for the drive motor PID calculations
        self.drivePIDController = wpimath.controller.PIDController(drivePID[0], drivePID[1], drivePID[2])
        print(*drivePID)

        # Gains for the rotate motot PID calculations
        self.rotatePIDController = wpimath.controller.ProfiledPIDController(rotatePID[0], rotatePID[1], rotatePID[2],
            wpimath.trajectory.TrapezoidProfile.Constraints(
            kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration))

        # Gains for drive and rotate motors
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        self.rotateFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 1.25)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.rotatePIDController.enableContinuousInput(-math.pi, math.pi)
        
        #Tell encoder that it is zeroed
        self.rotateEncoder.set_position(0)

        #Set the PID tolerance
        self.rotatePIDController.setTolerance(0.02)


    
    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module."""
        speed = self.driveMotor.get()
        return wpimath.kinematics.SwerveModuleState(speed,
            wpimath.geometry.Rotation2d(self.rotateEncoder.get_position().value_as_double * math.tau))

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current rotational position of the module.
        :returns:   The current position of the module.
        """
        distance = self.driveMotor.getEncoder().getPosition()
        return wpimath.kinematics.SwerveModulePosition(distance,
            wpimath.geometry.Rotation2d(self.rotateEncoder.get_position().value_as_double * math.tau))

    def setDrivePIDGains(self, values: tuple) -> None:
        """Set new PID values for the drive motor
        :param values:  Tuple with 3 Float numbers representing P I and D values
        """
        self.drivePIDController = wpimath.controller.PIDController(*values)

    def setRotatePIDGains(self, values: tuple) -> None:
        """Set new PID values for the rotate motor
        :param values:  Tuple with 3 Float numbers representing P I and D values
        """
        self.rotatePIDController = wpimath.controller.ProfiledPIDController(*values,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration ) )

    def setDesiredState(self, desiredState: wpimath.kinematics.SwerveModuleState):
        """Sets the desired state for the module.
        :param desiredState:    Desired state with speed and angle.
        """
        encoderRotation = wpimath.geometry.Rotation2d(self.rotateEncoder.get_position().value_as_double * math.tau)

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos()
        
        # Calculate the drive output from the drive PID controller
        driveOutput = self.drivePIDController.calculate(
            self.driveMotor.getClosedLoopRampRate(), state.speed)

        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller
        turnOutput = self.rotatePIDController.calculate(
            self.rotateEncoder.get_position().value_as_double * math.tau, (state.angle.radians()))
        turnFeedforward = self.rotateFeedforward.calculate(
            self.rotatePIDController.getSetpoint().velocity)

        # Two statements below set new targets for the Swerve drive motor controller
        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.rotateMotor.setVoltage(turnOutput + turnFeedforward)

    
