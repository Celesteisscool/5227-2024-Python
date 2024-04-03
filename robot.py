import wpilib
import wpilib.drive
import wpimath.kinematics
import wpimath.geometry
import wpimath.units
import math
from config import *
import drivetrain
import wpimath.filter
import rev
import phoenix6
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPipelineResult import *
from photonlibpy import photonPoseEstimator
import robotpy_apriltag
from wpilib import shuffleboard, Field2d



def shufflePID(nameArray,pidArray): 
    '''Function to take a array of names and an array of pid values and PUT them on the dashboard'''
    for i in range(0,3):
        wpilib.SmartDashboard.putNumber(nameArray[i], pidArray[i])

def unshufflePID(nameArray,pidArray):
    '''Function to take a array of names and an array of pid values and TAKE them from the dashboard'''
    for i in range(0,3):
        pidArray[i] = wpilib.SmartDashboard.getNumber(nameArray[i], 0)

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        """All code in this routine is used to setup the robot"""
        shufflePID(["FrLftRot-P","FrLftRot-I","FrLftRot-D"],FrontLeftRotatePID) # Put default pid values on the dashboard
        shufflePID(["FrRtRot-P","FrRtRot-I","FrRtRot-D"],FrontRightRotatePID)
        shufflePID(["RrLftRot-P","RrLftRot-I","RrLftRot-D"],RearLeftRotatePID)
        shufflePID(["RrRtRot-P","RrRtRot-I","RrRtRot-D"],RearRightRotatePID)

        self.Swerve = drivetrain.Drivetrain()

        self.RightShooting = rev.CANSparkMax(21, rev.CANSparkMax.MotorType.kBrushless) # Declare Shooting motors
        self.LeftShooting = rev.CANSparkMax(23, rev.CANSparkMax.MotorType.kBrushless)

        self.PickupMotor = wpilib.PWMVictorSPX(0) # Declare Pickup motors 
        self.PickupServo = wpilib.Servo(1)
        self.RingButton = wpilib.DigitalInput(0) # Declare Button 

        self.ClimbLeft = wpilib.VictorSP(6) # Declare climbers
        self.ClimbLeftInput = wpilib.DigitalInput(9)
        self.ClimbRight = wpilib.VictorSP(5)
        self.ClimbRightInput = wpilib.DigitalInput(8)

        self.AmpServoLeft  = wpilib.Servo(9) # Declare servos for Amp bar
        self.AmpServoRight = wpilib.Servo(8)

        # self.Lights = wpilib.PowerDistribution().setSwitchableChannel

        # Sets up Shuffleboard values (These can be later changed)
        wpilib.SmartDashboard.putNumber("Shooting Speed", 1) 
        wpilib.SmartDashboard.putNumber("Slow Shooting Speed", 0.195)
        wpilib.SmartDashboard.putNumber("Pickup Speed", -0.8)
        wpilib.SmartDashboard.putNumber("Pickup Belts Speed", 0.1)
        wpilib.SmartDashboard.putNumber("Angle Up", 0)
        wpilib.SmartDashboard.putNumber("Angle Down", 1)
        wpilib.SmartDashboard.putNumber("Amp Servo Up Left", 0.05)
        wpilib.SmartDashboard.putNumber("Amp Servo Up Right", 0)
        wpilib.SmartDashboard.putNumber("Amp Servo Down Left", 0.8)
        wpilib.SmartDashboard.putNumber("Amp Servo Down Right", 0.8)
        wpilib.SmartDashboard.putBoolean("Amp State", False)
        wpilib.SmartDashboard.putBoolean("Pickup State", False)

        wpilib.SmartDashboard.putBoolean("Hold Amp bar?", False)
        wpilib.SmartDashboard.putBoolean("Gyro on?", True)
        wpilib.SmartDashboard.getBoolean("Enable Gyro Reset?", True)

        wpilib.SmartDashboard.putNumber("AUTO", 2)

        self.LightsTimer = wpilib.Timer()

        self.CameraPickup = PhotonCamera("Microsoft_LifeCam_HD-3000")
        self.CameraDriver = PhotonCamera("MicroID_WC9")

        self.Lights = wpilib.PowerDistribution().setSwitchableChannel

        self.Gyro = phoenix6.hardware.Pigeon2(32)

        print("Robot is intialized") # Status message

    def robotPeriodic(self):
        """All code in this routine is run every loop in every mode"""
        wpilib.SmartDashboard.putBoolean("Button State", self.RingButton.get())

    def teleopInit(self):
        """All code in this routine is run once at the start of teleop """

        # Gets all values from Suffleboard, and saves them as vars
        self.ShootingSpeed = wpilib.SmartDashboard.getNumber("Shooting Speed", 1) 
        self.SlowShootingSpeed = wpilib.SmartDashboard.getNumber("Slow Shooting Speed", 0.2)
        self.PickupSpeed = wpilib.SmartDashboard.getNumber("Pickup Speed", -0.8)
        self.PickupBeltSpeed = wpilib.SmartDashboard.getNumber("Pickup Belts Speed", 0.1)
        self.AngleUp = wpilib.SmartDashboard.getNumber("Angle Up", 0)
        self.AngleDown = wpilib.SmartDashboard.getNumber("Angle Down", 1)
        
        self.AmpAngleUpLeft = wpilib.SmartDashboard.getNumber("Amp Servo Up Left", 0)
        self.AmpAngleUpRight = wpilib.SmartDashboard.getNumber("Amp Servo Up Right", 0)
        self.AmpAngleDownLeft = wpilib.SmartDashboard.getNumber("Amp Servo Down Left", 0.6)
        self.AmpAngleDownRight = wpilib.SmartDashboard.getNumber("Amp Servo Down Right", 0.6)

        self.HoldAmp = wpilib.SmartDashboard.getBoolean("Hold Amp bar?", False)
        self.EnableGyroReset = wpilib.SmartDashboard.getBoolean("Enable Gyro Reset?", True)

        self.IntakeState = True  # Sate of the pickup (True = Up, False = Down)
        self.HasRing = False # State of the ring button
        self.AmpState = False # State of our Amp bar (True = Up, False = Down)
        self.ServoState = False # Puts servo down


        self.LightsTimer.stop()
        self.LightsTimer.reset()

        self.FieldLayout = robotpy_apriltag.loadAprilTagLayoutField(robotpy_apriltag.AprilTagField(robotpy_apriltag.AprilTagField.k2024Crescendo))
        self.Field = [self.FieldLayout.getTags(),
                      self.FieldLayout.getFieldLength(),
                      self.FieldLayout.getFieldWidth()] 
        self.AprilTagLayout = photonPoseEstimator.AprilTagFieldLayout(self.Field[0],self.Field[1],self.Field[2])      
        self.PoseStrat = photonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
        self.RobotToCamera = wpimath.geometry.Transform3d(wpimath.units.inchesToMeters(10), # X offset from center (back to front)
                                                          wpimath.units.inchesToMeters(0),  # Y offset from center
                                                          wpimath.units.inchesToMeters(17.5), # Z offset from center (Vertical)
                                                          wpimath.geometry.Rotation3d(0,0,0))
        self.PoseEstimator = photonPoseEstimator.PhotonPoseEstimator(self.AprilTagLayout, self.PoseStrat, self.CameraDriver, self.RobotToCamera)
        
        print("Robot has started Teleop mode") # Status message

    def teleopPeriodic(self):
        """All code in this routine is run every loop in teleop mode """

        """ JOYSTICK INPUT """
        # The reason this is here is because it needs to be updated.
        self.RobotCentricDPad = Gamepad2.getPOV(0) # Get DPad position for Robot Centric control

        self.DriveX = wpimath.applyDeadband(Gamepad1.getRawAxis(0), 0.1) # Set up drive joystics with deadzones
        self.DriveY = wpimath.applyDeadband(Gamepad1.getRawAxis(1), 0.1)
        self.DriveRot = wpimath.applyDeadband(Gamepad2.getRawAxis(2), 0.1) # Dylan
        # self.DriveRot = wpimath.applyDeadband(Gamepad2.getRawAxis(0), 0.1) # Alex

        self.BreakButton = (Gamepad1.getRawButton(3) and Gamepad2.getRawButton(2)) # Button combo for breaking
        self.SlowButton = Gamepad1.getRawButton(1)
        self.FastButton = Gamepad2.getRawButton(1)
        
        self.ShootButton = Gamepad3.getRawButton(1)
        self.SlowShootButton = Gamepad3.getRawButton(3)
        self.SlowShootButtonReleased = Gamepad3.getRawButtonReleased(3)
        self.ToggleIntakeButton = Gamepad3.getRawButtonPressed(4)
        self.ReverseIntakeButton = Gamepad3.getRawButton(2)
        self.BeltDPad = Gamepad3.getPOV(0)

        self.ClimbLeftAxis = Gamepad3.getRawAxis(1)
        self.ClimbRightAxis = Gamepad3.getRawAxis(5)

        self.AmpBarButton = Gamepad3.getRawButtonPressed(6)

        self.ResetGyroButton = Gamepad1.getRawButtonPressed(2)
        self.ServoButton = Gamepad3.getRawButtonPressed(8)

        """ END OF JOYSTICK INPUT """

        """ SHUFFLEBOARD """
        # These also need to be updated.
        wpilib.SmartDashboard.putBoolean("Amp State", self.AmpState)
        wpilib.SmartDashboard.putBoolean("Pickup State", self.IntakeState) 
        self.GyroOn = wpilib.SmartDashboard.getBoolean("Gyro on?", True)
        """ END OF SHUFFLEBOARD """

        if  self.RobotCentricDPad == -1:
            '''Drives Field-Centric if DPad is NOT pressed'''
            if not self.BreakButton:
                if self.SlowButton: self.driveWithJoystick(True, self.DriveX*-0.5, self.DriveY*0.5, self.DriveRot*0.5) # Slow mode
                elif self.FastButton: self.driveWithJoystick(True, self.DriveX*-1, self.DriveY*1, self.DriveRot*1) # Fast mode
                else: self.driveWithJoystick(self.GyroOn, self.DriveX*-0.25, self.DriveY*0.25, self.DriveRot*0.25) # Normal mode
            elif self.BreakButton:
                '''Stops robot driving when break button is pressed'''
                self.driveWithJoystick(self.GyroOn, 0, 0, 0)
        elif not (self.RobotCentricDPad == -1):
            '''Drives Robot-Centric if DPad IS pressed'''
            if self.SlowButton: # Slow mode
                self.driveWithJoystick(False, math.cos(math.radians(self.RobotCentricDPad))*0.25, math.sin(math.radians(self.RobotCentricDPad))*-0.25, 0)
            else: # Normal mode
                self.driveWithJoystick(False, math.cos(math.radians(self.RobotCentricDPad))*0.5, math.sin(math.radians(self.RobotCentricDPad))*-0.5, 0)

        self.HasRing = self.RingButton.get() # Sets var to button status

        if self.HoldAmp:
            self.AmpState = self.AmpBarButton # Moves Amp bar based on bumper
        else:
            if self.AmpBarButton: self.AmpState = not self.AmpState # Inverts the value for our Amp bar state
            if self.SlowShootButtonReleased: self.AmpState = False # Puts down amp bar after shooting


        if self.AmpState == True: # Sets both Amp bar servos to the "Up" state
            self.AmpServoLeft.set(self.AmpAngleUpLeft)
            self.AmpServoRight.set((self.AmpAngleUpRight*-1)+1)
        elif self.AmpState == False: # Sets both Amp bar servos to the "Down" state
            self.AmpServoLeft.set(self.AmpAngleDownLeft)
            self.AmpServoRight.set((self.AmpAngleDownRight*-1)+1)

        if self.ToggleIntakeButton: self.IntakeState = not self.IntakeState # Inverts the value for our Intake state

        if self.BeltDPad == 0:
            self.LeftShooting.set(self.PickupBeltSpeed*-1)
            self.RightShooting.set(self.PickupBeltSpeed)
        if self.BeltDPad == 180:
            self.LeftShooting.set(self.PickupBeltSpeed)
            self.RightShooting.set(self.PickupBeltSpeed*-1)


        
        if self.IntakeState == True:
            '''If our Pickup is in the "Up" state'''
            self.PickupMotor.set(0) # Stops pickup motor

            if self.ShootButton: # Shoot at full speed
                if self.BeltDPad == -1: self.LeftShooting.set(self.ShootingSpeed*-1)
                if self.BeltDPad == -1: self.RightShooting.set(self.ShootingSpeed)
            elif self.SlowShootButton: # Shoot at slow speed
                if self.BeltDPad == -1: self.RightShooting.set(self.SlowShootingSpeed)
                if self.BeltDPad == -1: self.LeftShooting.set(self.SlowShootingSpeed*-1)
            else: # Stop shooters
                if self.BeltDPad == -1: self.LeftShooting.set(0)
                if self.BeltDPad == -1: self.RightShooting.set(0)

        else:
            '''If our Pickup is in the "Down" state'''
            if self.HasRing == False: # If we do NOT have a ring
                if self.ReverseIntakeButton: # Reverse pickup
                    self.PickupMotor.set(self.PickupSpeed*-1)
                    if self.BeltDPad == -1: self.LeftShooting.set(self.PickupBeltSpeed)
                    if self.BeltDPad == -1: self.RightShooting.set(self.PickupBeltSpeed*-1)
                else: # Forward pickup
                    self.PickupMotor.set(self.PickupSpeed)
                    if self.BeltDPad == -1: self.LeftShooting.set(self.PickupBeltSpeed*-1)
                    if self.BeltDPad == -1: self.RightShooting.set(self.PickupBeltSpeed)
            else: self.IntakeState = True # If we have a ring, set us to the "Up" State


            if self.ServoButton: self.ServoState = not self.ServoState

            if self.ServoState:    
                self.PickupServo.set(self.AngleUp) # Moves servo to the "Up" State
            else:
                self.PickupServo.set(self.AngleDown) # Moves servo to the "Down" State
        
        self.ClimbLeft.set(self.ClimbLeftAxis*1)
        self.ClimbRight.set(self.ClimbRightAxis*1)

        
        # self.setLights(self.Lights,self.LightsTimer, self.IntakeState, self.HasRing)
        if False:
            if self.ResetGyroButton:
                self.LatestResult = PhotonCamera.getLatestResult(self.CameraDriver)
                if self.CameraDriver.getLatestResult().hasTargets:
                    self.PoseUpdate = self.PoseEstimator.update(self.LatestResult)
                    if self.PoseUpdate != None:
                        self.CalculatedYaw = wpimath.units.radiansToDegrees(self.PoseUpdate.estimatedPose.rotation().angle)
                        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue: 
                            wpilib.SmartDashboard.putNumber("Latest Yaw", self.CalculatedYaw)
                            self.Gyro.set_yaw(self.CalculatedYaw)
                        else:
                            wpilib.SmartDashboard.putNumber("Latest Yaw", self.CalculatedYaw-180)
                            self.Gyro.set_yaw(self.CalculatedYaw-180)

        if False:
            self.LatestPose = self.UpdatePose()
            if self.LatestPose != None:
                self.Field2d = Field2d()
                self.Field2d.setRobotPose(self.LatestPose)
                wpilib.SmartDashboard.putData("Pose", self.Field2d)



    def UpdatePose(self):
        self.PoseUpdate = self.PoseEstimator.update(self.LatestResult)
        if (self.PoseUpdate != None) and (self.CameraDriver.getLatestResult().hasTargets):
            return self.PoseUpdate.estimatedPose.toPose2d()
        else:
            return None



    def setLights(self, Lights, Timer: wpilib.Timer, State, Flash):
        if Flash:
            if (Timer.get() - math.floor(Timer.get()) < 0.5):
                Lights(True)
            else:
                Lights(False)
        elif State:
            Lights(True)
        else:
            Lights(False)
        
    def driveWithJoystick(self, fieldRelative: bool, xAxis, yAxis, RotAxis):
        '''Function to drive us with a joystick. Also puts Swerve Position on the AdvantageScope dashboard'''
        xSpeed = (-xAxis*drivetrain.kMaxSpeed)
        ySpeed = (-yAxis*drivetrain.kMaxSpeed)
        rot    = (-RotAxis*drivetrain.kMaxSpeed)
        self.Swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(270))
        wpilib.SmartDashboard.putNumberArray("Swerve Dashboard", drivetrain.Drivetrain.SwerveDashboard(self.Swerve))
        # wpilib.SmartDashboard.putNumber("Swerve Gyro", drivetrain.Drivetrain.getGyro(drivetrain.Drivetrain))
        
    def autonomousInit(self):
        """Contains all the Intialization code for the robot entering Autonomous"""
        
        # Gets all values from Suffleboard, and saves them as vars
        self.ShootingSpeed = wpilib.SmartDashboard.getNumber("Shooting Speed", 1)
        self.PickupSpeed = wpilib.SmartDashboard.getNumber("Pickup Speed", -0.8)
        self.PickupBeltSpeed = wpilib.SmartDashboard.getNumber("Pickup Belts Speed", 0.1)
        self.AngleUp = wpilib.SmartDashboard.getNumber("Angle Up", 0)
        self.AngleDown = wpilib.SmartDashboard.getNumber("Angle Down", 1)

        self.IntakeState = True  # Sate of the pickup (True = Up, False = Down)
        self.HasRing = False # State of the ring button
        self.AmpState = False # State of our Amp bar (True = Up, False = Down)

        self.PickingUp = False

        self.TimerOne = wpilib.Timer() # Sets up timers for auto
        self.TimerTwo = wpilib.Timer()
        self.TimerOne.reset() 
        self.TimerTwo.reset()

        self.TimerOne.start() # Starts one timer, other is started within Auto

        self.AutoPos = wpilib.SmartDashboard.getNumber("AUTO", 2)

        print("Robot has started Autonomous mode") # Status message

    def autonomousPeriodic(self):
        wpilib.SmartDashboard.putNumber("Timer One", self.TimerOne.get()) # Puts timers on dashboard for debuging
        wpilib.SmartDashboard.putNumber("Timer Two", self.TimerTwo.get())

        if self.AutoPos == 2:
            if self.TimerOne.get() < BlueTwoT1:
                self.LeftShooting.set(self.ShootingSpeed*-1)
                self.RightShooting.set(self.ShootingSpeed)
            if BlueTwoT1 <= self.TimerOne.get() < BlueTwoT2:
                self.Swerve.drive(-0.1, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
                self.LeftShooting.set(0)
                self.RightShooting.set(0)
            if BlueTwoT2 <= self.TimerOne.get() < BlueTwoT3:
                self.Swerve.drive(-0.1, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
                # self.PickupServo.set(self.AngleDown)
                self.PickupMotor.set(self.PickupSpeed)
                self.LeftShooting.set(0.075*-1)
                self.RightShooting.set(0.075)
                self.PickingUp = True

            if BlueTwoT3 <= self.TimerOne.get() < BlueTwoT4:
                self.Swerve.drive(0, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
                # self.PickupServo.set(self.AngleDown)
                self.PickupMotor.set(self.PickupSpeed)
                self.LeftShooting.set(0.075*-1)
                self.RightShooting.set(0.075)

            if (self.RingButton.get() == True) and (self.PickingUp == True):
                self.TimerTwo.start()
                self.PickingUp = False

            if BlueTwoT5 < self.TimerTwo.get() < BlueTwoT6:
                self.Swerve.drive(0.1, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
                # self.PickupServo.set(self.AngleUp)
                self.PickupMotor.set(0)
                self.LeftShooting.set(0)
                self.RightShooting.set(0)
            if BlueTwoT6 <= self.TimerTwo.get() < BlueTwoT7:
                self.Swerve.drive(0, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
            if BlueTwoT7 <= self.TimerTwo.get() < BlueTwoT8:
                self.LeftShooting.set(self.ShootingSpeed*-1)
                self.RightShooting.set(self.ShootingSpeed)
            if BlueTwoT8 <= self.TimerTwo.get():
                self.LeftShooting.set(0)
                self.RightShooting.set(0)
                self.TimerOne.stop()
                self.TimerTwo.stop()

        if (self.AutoPos == 1):
            self.PickupMotor.set(0)
            if 0.01 < self.TimerOne.get() < BlueOneT1:
                self.Swerve.drive(-0.1, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
            if  BlueOneT1 < self.TimerOne.get() < BlueOneT2:
                self.Swerve.drive(0, 0, 0.1, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
            if BlueOneT2 < self.TimerOne.get() < BlueOneT3:
                self.Swerve.drive(0.1, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
            if BlueOneT3 < self.TimerOne.get() < BlueOneT4:
                self.Swerve.drive(0, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
            if BlueOneT4 < self.TimerOne.get() < BlueOneT5:
                self.LeftShooting.set(self.ShootingSpeed*-1)
                self.RightShooting.set(self.ShootingSpeed)
            if BlueOneT5 < self.TimerOne.get():
                self.LeftShooting.set(0)
                self.RightShooting.set(0)
                self.TimerOne.stop()
                self.TimerTwo.stop()

        if (self.AutoPos == 3):
            self.PickupMotor.set(0)
            if 0.01 < self.TimerOne.get() < BlueOneT1:
                self.Swerve.drive(-0.1, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
            if  BlueOneT1 < self.TimerOne.get() < BlueOneT2:
                self.Swerve.drive(0, 0, -0.1, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
            if BlueOneT2 < self.TimerOne.get() < BlueOneT3:
                self.Swerve.drive(0.1, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
            if BlueOneT3 < self.TimerOne.get() < BlueOneT4:
                self.Swerve.drive(0, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
            if BlueOneT4 < self.TimerOne.get() < BlueOneT5:
                self.LeftShooting.set(self.ShootingSpeed*-1)
                self.RightShooting.set(self.ShootingSpeed)
            if BlueOneT5 < self.TimerOne.get():
                self.LeftShooting.set(0)
                self.RightShooting.set(0)
                self.TimerOne.stop()
                self.TimerTwo.stop()

        if (self.AutoPos == 4):
            if 0.01 < self.TimerOne.get() < BlueThreeT1:
                self.Swerve.drive(-0.1, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))
            if BlueThreeT1 < self.TimerOne.get():
                self.Swerve.drive(0, 0, 0, False, self.getPeriod(), wpimath.geometry.Rotation2d.fromDegrees(180))



    def testInit(self):
        """Contains all the Intialization code for the robot entering Test Mode"""
        print("Robot is in test mode")
        self.Rotation = 0
        self.Speed = 0
        
    def testPeriodic(self):
        """Contains all the code for the robot while in Test Mode"""
        # CODE MIGHT NOT WORK
        if Gamepad3.getRawButtonPressed(3):
            self.Rotation -= (math.pi / 4)
        if Gamepad3.getRawButtonPressed(2):
            self.Rotation += (math.pi / 4)
        if Gamepad3.getRawButtonPressed(5):
            self.Rotation = 0
            print("Zeroed")

        drivetrain.Drivetrain.testMode(self.Swerve, wpimath.geometry.Rotation2d(self.Rotation))
        wpilib.SmartDashboard.putNumberArray("SwerveDashboard", drivetrain.Drivetrain.SwerveDashboard(self.Swerve))

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        print("Robot is disabled")

    def disabledPeriodic(self) -> None:
        # CODE MIGHT NOT WORK
        """This function is called periodically when disabled"""
        unshufflePID(["FrLftRot-P","FrLftRot-I","FrLftRot-D"],FrontLeftRotatePID)
        unshufflePID(["FrRtRot-P","FrRtRot-I","FrRtRot-D"],FrontRightRotatePID)
        unshufflePID(["RrLftRot-P","RrLftRot-I","RrLftRot-D"],RearLeftRotatePID)
        unshufflePID(["RrRtRot-P","RrRtRot-I","RrRtRot-D"],RearRightRotatePID)
    
        if Gamepad3.getRawButtonPressed(5):
            drivetrain.Drivetrain.updatePID(self.Swerve)
            print("PID values updated")
        
        # if Gamepad3.getRawButtonPressed(7):
        #     self.PoseMode = photonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
        #     self.LatestResult = PhotonCamera.getLatestResult(self.CameraPickup)
        #     if self.LatestResult.hasTargets():
        #         self.RobotPose = photonPoseEstimator.PhotonPoseEstimator.update(self.PoseMode,self.CameraDriver.getLatestResult()).estimatedPose
        #         wpilib.SmartDashboard.putNumber("RobotRotApril", wpimath.units.degrees(wpimath.geometry.Pose3d.rotation(self.RobotPose).angle_degrees))
        #    if abs(self.LatestResult.getTargets()[0].getYaw - (self.Gyro.get_yaw().value_as_double, 360)) > 5:
        #         self.Swerve.drive(0,0,(self.LatestResult.getTargets()[0].getYaw - self.Gyro.get_yaw().value_as_double/180))

        # if len(self.Camera.getLatestResult().getTargets()) > 0:
        #     self.Lights(True)
        #     wpilib.SmartDashboard.putBoolean("Has Targets", True)
        # else:
        #     self.Lights(False)
        #     wpilib.SmartDashboard.putBoolean("Has Targets", False)
        
        # wpilib.SmartDashboard.putNumber("ID", wpilib.PowerDistribution.getVoltage())