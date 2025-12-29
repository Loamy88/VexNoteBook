#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
brain_inertial = Inertial()



# generating and setting random seed
def initializeRandomSeed():
    wait(100, MSEC)
    xaxis = brain_inertial.acceleration(XAXIS) * 1000
    yaxis = brain_inertial.acceleration(YAXIS) * 1000
    zaxis = brain_inertial.acceleration(ZAXIS) * 1000
    systemTime = brain.timer.system() * 100
    urandom.seed(int(xaxis + yaxis + zaxis + systemTime)) 
    
# Initialize random seed 
initializeRandomSeed()

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      VEXcode Project
# 	Author:       VEX
# 	Created:
# 	Description:  VEXcode IQ Python Project
# 
# ------------------------------------------


print("[DEBUG] Starting...")
# Library imports
from vex import *


# Globals
DriveMode = "tank"
DoingSequence = []


# Initialize Motors

class Init:
    def __init__(self):
        if brain.battery.capacity() <= 75:
            brain.screen.print("Battery low")
        if brain.buttonLeft.pressing():
            self.Debug = True
            brain.screen.print("Debug Mode")
        else:
            self.Debug = False
        self.Control = Controller()
        self.BeamArm = MotorGroup(Motor(Ports.PORT7), Motor(Ports.PORT1, True))
        self.BeamArm.set_stopping(HOLD)
        self.Claws = Pneumatic(Ports.PORT11)
        self.PinArm = MotorGroup(Motor(Ports.PORT10), Motor(Ports.PORT4, True))
        self.PinArm.set_stopping(HOLD)
        self.PinArm.set_position(0, DEGREES)
        #  - Simplicity for controlling the claws -
        self.Beam = CYLINDER1
        self.Pin = CYLINDER2
        for motor in [self.BeamArm, self.PinArm]:
            motor.set_velocity(100, PERCENT)
        self.DriveMotors = self.InitDrive()
    class InitDrive:
        def __init__(self):
            self.Left = Motor(Ports.PORT9, True)
            self.Right = Motor(Ports.PORT3)
            self.Main = MotorGroup(self.Left, self.Right)
            self.Main.set_velocity(100, PERCENT)
    class InitPID:
        def __init__(self, kP, kI, kD, WheelDiameter):
            print("[DEBUG] Initializing PID Controller...", end="")
            self.k = [kP, kI, kD]
            self.DegreesPerInch = 360 / (WheelDiameter * math.pi)
            print("  ...PID Initialized")
        def Drive(self, Direction, RightMotor, LeftMotor, TargetDistance, Velocity, k):
            TargetPos = self.DegreesPerInch * TargetDistance
            kP = k[0]
            kI = k[1]
            kD = k[2]
            RightIntegral = 0
            LeftIntegral = 0
            RightLastError = 0
            LeftLastError = 0
            while True:
                RightPos = RightMotor.position(DEGREES)
                LeftPos = LeftMotor.position(DEGREES)
                RightError = TargetPos - RightPos
                LeftError = TargetPos - LeftPos
                RightIntegral += RightError
                LeftIntegral += LeftError
                RightDerivative = RightError - RightLastError
                LeftDerivative = LeftError - LeftLastError
                RightLastError = RightError
                LeftLastError = LeftError

                RightPower = (RightError * kP) + (RightDerivative * kD) + (RightIntegral * kI)
                LeftPower = (LeftError * kP) + (LeftDerivative * kD) + (LeftIntegral * kI)
                RightMotor.set_velocity(Clamp(RightPower), PERCENT)
                LeftMotor.set_velocity(Clamp(LeftPower), PERCENT)
                RightMotor.spin(Direction)
                LeftMotor.spin(Direction)

                if abs(RightError) < 7 and abs(LeftError) < 7:
                    RightMotor.stop()
                    LeftMotor.stop()
                    return

                wait(20, MSEC)
        def SpinMotor(self, Direction, Motor_, Distance_, k):
            kP = k[0]
            kI = k[1]
            kD = k[2]
            Integral = 0
            LastError = 0
            while True:
                Pos = Motor_.position(DEGREES)
                Error = Distance_ - Pos
                Integral += Error
                Derivative = Error - LastError
                LastError = Error

                Power = (Error * kP) + (Derivative * kD) + (Integral * kI)
                Motor_.set_velocity(Clamp(Power), PERCENT)
                Motor_.spin(Direction)

                if abs(Error) < 7:
                    Motor_.stop()
                    return

                wait(20, MSEC)



Robot = Init()
Robot.PID = Robot.InitPID(0.4, 0.0006, 0.25, 3)

def Clamp(Num, Max=100, Min=-100):
    return max(min(Num, Max), Min)

def Drive():
    global DriveMode
    while True:
        a = Robot.Control.axisA.position()
        c = Robot.Control.axisC.position()
        d = Robot.Control.axisD.position()
        if DriveMode == "tank":
            LeftSide = a
            RightSide = d
        elif DriveMode == "arcade":
            LeftSide = Clamp((a + c))
            RightSide = Clamp((a - c))
        Sides = [Robot.DriveMotors.Right, Robot.DriveMotors.Left]
        # Check if the speed is in the deadband range.
        if RightSide < 6 and RightSide > -6:
            Sides[0].stop()
        else:
            # Spin the side at the speed 0-100 in the correct direction.
            Sides[0].set_velocity(abs(RightSide), PERCENT)
            Sides[0].spin(FORWARD if RightSide > 0 else REVERSE)
        # Check if the speed is in the deadband range.
        if LeftSide < 6 and LeftSide > -6:
            Sides[1].stop()
        else:
            # Spin the side at the speed 0-100 in the correct direction.
            Sides[1].set_velocity(abs(LeftSide), PERCENT)
            Sides[1].spin(FORWARD if LeftSide > 0 else REVERSE)
        while "drive" in DoingSequence:
            wait(30, MSEC)
        if Robot.Debug:
            print("Left:", Robot.DriveMotors.Left.velocity(PERCENT), "  Right:", Robot.DriveMotors.Right.velocity(PERCENT))

def ClawControl():
    while True:
        if Robot.Control.buttonFDown.pressing(): # Beam claw open
            Robot.Claws.retract(Robot.Beam)
        if Robot.Control.buttonFUp.pressing(): # Beam claw close
            Robot.Claws.extend(Robot.Beam)
        if Robot.Control.buttonEDown.pressing(): # Pin claw open
            Robot.Claws.retract(Robot.Pin)
        if Robot.Control.buttonEUp.pressing(): # Pin claw close
            Robot.Claws.extend(Robot.Pin)

def ArmControl():
    global DoingSequence
    Delay = 0
    while True:
        if Robot.Control.buttonRUp.pressing(): # Lift beam arm
            Robot.BeamArm.spin(FORWARD)
        if Robot.Control.buttonRDown.pressing(): # Lower beam arm
            Robot.BeamArm.spin(REVERSE)
        if not Robot.Control.buttonRUp.pressing() and not Robot.Control.buttonRDown.pressing() and not "beam" in DoingSequence: # Check for beam arm not moving
            Robot.BeamArm.stop()
        if Robot.Control.buttonLUp.pressing(): # Lift pin arm
            if (Robot.PinArm.position(DEGREES) / 3) > 90 and Robot.Debug: # disable to stop auto slow down
                    pos = (Robot.PinArm.position(DEGREES) / 3) - 85
                    Robot.PinArm.set_velocity(Clamp(round(100 - (1.045 ** pos)), 0, 100), PERCENT)
                    print(pos, ": ", round(100 - (1.045 ** pos)), sep="")
            Robot.PinArm.spin(FORWARD)
        if Robot.Control.buttonLDown.pressing(): # Lower pin arm
            Robot.PinArm.spin(REVERSE)
            if Robot.PinArm.position(DEGREES) < -5:
                Robot.PinArm.set_position(0, DEGREES)
            if Robot.Debug: # disable to stop auto releasing pins
                if Robot.PinArm.velocity(PERCENT) < 5:
                    if Delay < 100:
                        Delay += 1
                    else:
                        Robot.Claws.retract(Robot.Pin)
                else:
                    Delay = 0
        else:
            Delay = 0
        if not Robot.Control.buttonLUp.pressing() and not Robot.Control.buttonLDown.pressing() and not "pin" in DoingSequence: # Check for pin arm not moving
            Robot.PinArm.stop()

def SwitchModes():
    global DriveMode
    DriveMode = "tank" if DriveMode == "arcade" else "arcade"


def PlaceStack():
    global DoingSequence
    DoingSequence.append("pin", "drive")
    Robot.DriveMotors.Main.set_velocity(20, PERCENT)
    Robot.DriveMotors.Main.spin(REVERSE)
    wait(240, MSEC)
    Robot.DriveMotors.Main.set_velocity(100, PERCENT)
    # Places pins in the claw onto pins in the current slots
    Robot.DriveMotors.Main.spin(FORWARD)
    Robot.PinArm.spin(REVERSE)
    wait(400, MSEC)
    Robot.Claws.retract(Robot.Pin)
    Robot.DriveMotors.Main.stop()
    Robot.PinArm.stop()
    DoingSequence.remove("drive")
    DoingSequence.remove("pin")

def Num2Let(n):
    result = ''
    while n > 0:
        n -= 1
        n, remainder = divmod(n, 26)
        result = chr(97 + remainder) + result
    return result

def main():
    sd = brain.sdcard
    VidTimer = Timer()
    try:
        DriveThread = Thread(Drive)
        ArmThread = Thread(ArmControl)
        ClawThread = Thread(ClawControl)
        Robot.Control.buttonR3.pressed(SwitchModes)
        Stop = brain.program_stop
        Robot.Control.buttonL3.pressed(Stop)
        if sd.is_inserted():
            print("[DEBUG] SD Card Found, Starting Saved Video")
            if sd.exists("Videos\\Merica6FPS\\a.bmp"):
                brain.screen.render()
                WaitTime = 1000 / 6
                while True:
                    i = 0
                    Video = True
                    while Video:
                        i += 1
                        VidTimer.clear()
                        FileName = "Videos\\Merica6FPS\\" + Num2Let(i) + ".bmp"
                        if sd.exists(FileName):
                            brain.screen.clear_screen()
                            brain.screen.draw_image_from_file(FileName, 40, 32)
                            brain.screen.render()
                            while VidTimer.time() < WaitTime:
                                pass
                        else:
                            print("[DEBUG] Ended Video")
                            Video = False
            else:
                print("[ERROR] Video Files Not Found")
    except Exception as e:
        print("[ERROR]", e)

main()
