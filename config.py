import wpilib

# PID Tuning variables
FrontLeftDrivePID   = [0, 0, 0]
FrontRightDrivePID  = [0, 0, 0]
RearLeftDrivePID    = [0, 0, 0]
RearRightDrivePID   = [0, 0, 0]

FrontLeftRotatePID  = [6, 0, 0]
FrontRightRotatePID = [6, 0, 0]
RearLeftRotatePID   = [6, 0, 0]
RearRightRotatePID  = [6, 0, 0]

# Gamepads + Buttons:
Gamepad1 = wpilib.Joystick(0)
Gamepad2 = wpilib.Joystick(1)
Gamepad3 = wpilib.Joystick(2)

# Sides
BlueOneT1 = 1.6 # Back up
BlueOneT2 = BlueOneT1 + 1.62 # Rotate
BlueOneT3 = BlueOneT2 + 1.9 # Move to speaker
BlueOneT4 = BlueOneT3 + 0.5 # Pause
BlueOneT5 = BlueOneT4 + 0.75 # Shoot
BlueOneT6 = BlueOneT5 + 0.5 # Stop

# Center
BlueTwoT1 = 0.75 # Shoot
BlueTwoT2 = BlueTwoT1 + 0.5 # Move to another ring
BlueTwoT3 = BlueTwoT2 + 2.75 # Put down intake
BlueTwoT4 = BlueTwoT3 + 1 # Stop moving
# Waits until we have ring
BlueTwoT5 = 0.01 # KEEP AT 0.01!!!!!!!
BlueTwoT6 = BlueTwoT5 + 3.25 # Move to shoot
BlueTwoT7 = BlueTwoT6 + 1.25 # Drift into place
BlueTwoT8 = BlueTwoT7 + 0.5 # Shoot

BlueThreeT1 = 5