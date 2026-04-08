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

print("\033[2J\n\033[34m-====-   Program Start   -====-\033[0m\n")

DEBUG = "\033[35m[DEBUG]\033[0m"
ERROR = "\033[31m[ERROR]\033[0m"


print(DEBUG, "Starting...")
# Library imports

version = "0.2.1"

print(DEBUG, "New 180 Flip Code Version:", version)




# Globals
DriveMode = "tank"
DoingSequence = []
LiftingBeam = None
Aligned = False
ActivePinAligner = True
Flipping = None
Lifting = None


# Initialize Devices

class Init:
    def __init__(self):
        self.LowBat()
        print(DEBUG, "Initilizing Devices")
        if brain.buttonLeft.pressing():
            self.Debug = True
            brain.screen.print("Debug Mode")
        else:
            self.Debug = False
        self.Control = Controller()
        self.IsStopping = False
        self.Light = Touchled(Ports.PORT6)
        self.Light2 = Touchled(Ports.PORT7)
        self.BeamArm = MotorGroup(Motor(Ports.PORT10), Motor(Ports.PORT1, True))
        self.BeamArm.set_stopping(HOLD)
        self.AlignerPinClaw = Pneumatic(Ports.PORT9)
        self.PinArm = MotorGroup(Motor(Ports.PORT11), Motor(Ports.PORT12, True))
        self.PinArm.set_stopping(HOLD)
        self.PinArm.set_position(0, DEGREES)
        #  - Simplicity for controlling the claws -
        self.Beam = CYLINDER2
        self.BeamBinary = 3 
        self.Pin = CYLINDER2
        self.PinBinary = 3
        # CYLINDER1 = 768
        self.ClawMode = "TWOBUTTONOPENCLOSE"
        for motor in [self.BeamArm, self.PinArm]:
            motor.set_velocity(100, PERCENT)
        self.DriveMotors = self.InitDrive()
        self.PASBeamClaw = Pneumatic(Ports.PORT5)
        self.Aligner = self.InitAligner(self.AlignerPinClaw)
        self.PinAligner = self.InitPAS(self.PASBeamClaw)
        self.Light.on()
        self.Light.set_color(Color.RED)
        self.Light2.on()
        self.Light2.set_color(Color.RED)
        self.L3 = False
        self.Control.buttonL3.pressed(self.PressL3)
        self.Control.buttonL3.released(self.ReleaseL3)
        print(DEBUG, "Devices Initilized   ")
    def PressL3(self):
        self.L3 = True
    def ReleaseL3(self):
        wait(100, MSEC)
        if not self.Control.buttonL3.pressing():
            self.L3 = False
    def LowBat(self):
        if brain.battery.capacity() <= 75:
            print(DEBUG, "Low Battery:", brain.battery.capacity())
            Percent = brain.battery.capacity() / 100
            brain.screen.set_fill_color(Color.RED)
            brain.screen.set_pen_color(Color.RED)
            brain.screen.draw_rectangle(40, 29, 81 * Percent, 52)
            brain.screen.set_pen_width(6)
            brain.screen.set_fill_color(Color.TRANSPARENT)
            brain.screen.set_pen_color(Color.WHITE)
            brain.screen.draw_rectangle(34, 23, 90, 60)
            brain.screen.set_fill_color(Color.WHITE)
            brain.screen.set_pen_width(1)
            brain.screen.draw_rectangle(27, 43, 8, 24)
    class InitDrive:
        def __init__(self):
            self.Left = Motor(Ports.PORT2)
            self.Right = Motor(Ports.PORT8, True)
            self.Main = MotorGroup(self.Left, self.Right)
            self.Main.set_velocity(100, PERCENT)
    class InitAligner:
        def __init__(self, Pn):
            self.PneumaticDevice = Pn
        def down(self):
            self.PneumaticDevice.extend(CYLINDER1)
        def up(self):
            self.PneumaticDevice.retract(CYLINDER1)
    class InitPAS:
        def __init__(self, Pn):
            self.PneumaticDevice = Pn
        def down(self):
            self.PneumaticDevice.extend(CYLINDER1)
        def up(self):
            self.PneumaticDevice.retract(CYLINDER1)
    class InitPID:
        def __init__(self, kP, kI, kD, WheelDiameter):
            print(DEBUG, "Initializing PID Controller")
            self.k = [kP, kI, kD]
            self.DegreesPerInch = 360 / (WheelDiameter * math.pi)
            print(DEBUG, "PID Initialized")
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


print("\n\033[34m---- Initilizing ----\n\033[0m")

Robot = Init()
# Robot.PID = Robot.InitPID(0.4, 0.0006, 0.25, 3)

print("\n\033[34m---- Initilization Complete ----\n\033[0m")

def Clamp(Num, Max=100, Min=-100):
    return max(min(Num, Max), Min)


def Drive():
    global DriveMode
    while True:
        wait(8, MSEC)
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
            Sides[0].set_velocity(RightSide, PERCENT)
            Sides[0].spin(FORWARD)
        # Check if the speed is in the deadband range.
        if LeftSide < 6 and LeftSide > -6:
            Sides[1].stop()
        else:
            # Spin the side at the speed 0-100 in the correct direction.
            Sides[1].set_velocity(LeftSide, PERCENT)
            Sides[1].spin(FORWARD)
        while "drive" in DoingSequence:
            wait(30, MSEC)

def ClawAlignerControl():
    FUpReady = True
    FDownReady = True
    EUpReady = True
    EDownReady = True


    
    if Robot.ClawMode == "ONEBUTTONTOGGLE":
        while True:
            wait(8, MSEC)

            # ----- Checking Claw -----
            #
            # pump: 1000 0000 0000 0000 (65536)
            # cyninder 1: 0000 0000 0011 0000 0000 (768)
            # cylinder 2: 0000 0000 0000 0000 0011 (3)
            #
            # Ex. 1000 0000 0100 0000 0100 (claws.status() = 66564)

            if Robot.Control.buttonFDown.pressing(): # Beam claw toggle
                if FDownReady:
                    if Robot.Debug:
                        print(bin(Robot.PASBeamClaw.status()))
                    if Robot.PASBeamClaw.status() & Robot.BeamBinary == Robot.BeamBinary:
                        # Check if claw is open with returned bits from pneumatic status
                        Robot.PASBeamClaw.retract(Robot.Beam)
                    else:
                        Robot.PASBeamClaw.extend(Robot.Beam)
                    FDownReady = False
            else:
                FDownReady = True


            if Robot.Control.buttonFUp.pressing(): # Beam aligner toggle
                if FUpReady:
                    ActivateAligner(NoL3=True)
                    FUpReady = False
            else:
                FUpReady = True

                
            if Robot.Control.buttonEDown.pressing(): # Pin claw toggle
                if EDownReady:
                    if Robot.AlignerPinClaw.status() & Robot.PinBinary == Robot.PinBinary:
                        # Check if claw is open with returned bits from pneumatic status
                        Robot.AlignerPinClaw.retract(Robot.Pin)
                    else:
                        Robot.AlignerPinClaw.extend(Robot.Pin)
                    EDownReady = False
            else:
                EDownReady = True


            if Robot.Control.buttonEUp.pressing(): # Pin aligner toggle
                if EUpReady:
                    ActivatePAS(NoL3=True)
                    EUpReady = False
            else:
                EUpReady = True


    elif Robot.ClawMode == "TWOBUTTONOPENCLOSE":
        while True:
            # - Secondary Mode (Not in use) -

            wait(8, MSEC)
            if Robot.Control.buttonFDown.pressing(): # Beam claw open
                Robot.PASBeamClaw.retract(Robot.Beam)
            if Robot.Control.buttonFUp.pressing(): # Beam claw close
                Robot.PASBeamClaw.extend(Robot.Beam)
            if Robot.Control.buttonEDown.pressing(): # Pin claw open
                Robot.AlignerPinClaw.retract(Robot.Pin)
            if Robot.Control.buttonEUp.pressing(): # Pin claw close
                Robot.AlignerPinClaw.extend(Robot.Pin)

def ArmControl():
    global DoingSequence, Flipping, Lifting
    Delay = 0
    while True:
        wait(8, MSEC)
        if not Robot.L3:
            if Robot.Control.buttonRUp.pressing(): # Lift beam arm
                if Lifting:
                    Lifting.stop()
                    Lifting = None
                    Robot.BeamArm.set_velocity(100, PERCENT)
                Robot.BeamArm.spin(FORWARD)
            if Robot.Control.buttonRDown.pressing(): # Lower beam arm
                if Lifting:
                    Lifting.stop()
                    Lifting = None
                    Robot.BeamArm.set_velocity(100, PERCENT)
                Robot.BeamArm.spin(REVERSE)
            if Robot.BeamArm.position(DEGREES) < 0:
                Robot.BeamArm.reset_position()
            elif Robot.BeamArm.position(DEGREES) < 58:
                Robot.BeamArm.set_stopping(COAST)
            else:
                Robot.BeamArm.set_stopping(HOLD)

            if not Robot.Control.buttonRUp.pressing() and not Robot.Control.buttonRDown.pressing() and not "beam" in DoingSequence: # Check for beam arm not moving
                if not Lifting:
                    Robot.BeamArm.stop()
            if Robot.Control.buttonLUp.pressing(): # Lift pin arm
                if Flipping:
                    Flipping.stop()
                    Flipping = None
                Robot.PinArm.spin(FORWARD)
            if Robot.Control.buttonLDown.pressing(): # Lower pin arm
                Robot.PinArm.spin(REVERSE)
                if Flipping:
                    Flipping.stop()
                    Flipping = None
            if Robot.PinArm.position(DEGREES) < 115:
                Robot.PASBeamClaw.extend(CYLINDER1)
            elif Robot.PinArm.position(DEGREES) > 130:
                Robot.PASBeamClaw.retract(CYLINDER1)
            if Robot.PinArm.position(DEGREES) < 0:
                Robot.PinArm.reset_position()
            elif Robot.PinArm.position(DEGREES) < 50:
                Robot.PinArm.set_stopping(COAST)
            else:
                Robot.PinArm.set_stopping(HOLD)

            if not Robot.Control.buttonLUp.pressing() and not Robot.Control.buttonLDown.pressing() and not "pin" in DoingSequence: # Check for pin arm not moving
                if not Flipping:
                    Robot.PinArm.stop()

def SwitchModes():
    global DriveMode
    DriveMode = "tank" if DriveMode == "arcade" else "arcade"
    if DriveMode == "tank":
        Robot.Light.set_color(Color.RED)
        Robot.Light2.set_color(Color.RED)
    else:
        Robot.Light.set_color(Color.BLUE)
        Robot.Light2.set_color(Color.BLUE)

def ActivateAligner(NoL3=False):
    global Aligned
    if Robot.L3 or NoL3:
        if Aligned:
            Robot.Aligner.up()
            Aligned = False
        else:
            Robot.Aligner.down()
            Aligned = True

def ActivatePAS(NoL3=False):
    global ActivePinAligner
    if Robot.L3 or NoL3:
        if ActivePinAligner:
            Robot.PinAligner.up()
            ActivePinAligner = False
        else:
            Robot.PinAligner.down()
            ActivePinAligner = True

'''
def RunAutoFlip():
    global Flipping
    if Robot.PinArm.position(DEGREES) / 3 > 150:
        direction = REVERSE
        def check():
            return Robot.PinArm.position(DEGREES) / 3 > 30
    else:
        direction = FORWARD
        def check():
            return Robot.PinArm.position(DEGREES) / 3 < 150
    Robot.PinArm.spin(direction)
    while check():
        wait(150, MSEC)
    wait(400, MSEC)
    Robot.PinArm.stop()
    Flipping = None

def AutoFlip():
    global Flipping
    if Robot.L3:
        if Flipping:
            Flipping.stop()
            Flipping = None
        else:
            Flipping = Thread(RunAutoFlip)

def RunAutoLift():
    global Lifting
    if Robot.BeamArm.position(DEGREES) / 5 > 45:
        direction = REVERSE
        target = 30
        def check():
            return Robot.BeamArm.position(DEGREES) / 5 > 30
    else:
        direction = FORWARD
        target = 100
        def check():
            return Robot.BeamArm.position(DEGREES) / 5 < 100
    Robot.BeamArm.set_velocity(100, PERCENT)
    Robot.BeamArm.spin(direction)
    if direction == FORWARD:
        while check():
            wait(20, MSEC)
            power = min((abs(target - Robot.BeamArm.position(DEGREES) / 5) + 30) * 1.05, 100)
            Robot.BeamArm.set_velocity(power, PERCENT)
            Robot.BeamArm.spin(direction)
    else:
        while check():
            wait(150, MSEC)
    wait(400, MSEC)
    Robot.BeamArm.stop()
    Robot.BeamArm.set_velocity(100, PERCENT)
    Lifting = None

def AutoLift():
    global Lifting
    if Robot.L3:
        if Lifting:
            Lifting.stop()
            Lifting = None
        else:
            Lifting = Thread(RunAutoLift)
'''

def StopCheck():
    Pressing = False
    Count = 0
    while True:
        wait(50, MSEC)
        if Robot.Control.buttonR3.pressing() and Robot.Control.buttonL3.pressing():
            Count += 1
            Robot.IsStopping = True
        else:
            Count = 0
            Robot.IsStopping = False
        if Count == 40:
            Robot.Aligner.up()
            brain.program_stop()
        if Count == 20:
            brain.play_note(2, 3, 400)
            Robot.Aligner.up()
            Robot.LowBat()

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
        ClawAlignerThread = Thread(ClawAlignerControl)
        Robot.Control.buttonR3.pressed(SwitchModes)
        Robot.Control.buttonRDown.pressed(ActivateAligner)
        Robot.Control.buttonLDown.pressed(ActivatePAS)
        Robot.Control.buttonLUp.pressed(ActivatePAS)
        Robot.Control.buttonRUp.pressed(ActivateAligner)
        StopThread = Thread(StopCheck)
        if sd.is_inserted():
            print(DEBUG, "SD Card Found, Tracking Movement")
            try:
                if not sd.exists("TravelDistance.txt"):
                    sd.savefile("TravelDistance.txt", bytearray("0"))
                    current_distance = 0
                else:
                    try:
                        current_distance = int(sd.loadfile("TravelDistance.txt"))
                    except Exception:
                        sd.savefile("TravelDistance.txt", bytearray("0"))
                        current_distance = 0
                previous_positions = (0, 0)
                while sd.is_inserted():
                    if not (brain.buttonCheck.pressing() or Robot.IsStopping):

                        change = 0
                        current_positions = (Robot.DriveMotors.Right.position(DEGREES), Robot.DriveMotors.Left.position(DEGREES))

                        for side_index, side_position in enumerate(current_positions):
                            change += side_position - previous_positions[side_index]
                        
                        change /= 2

                        current_distance += abs(change)
                        sd.savefile("TravelDistance.txt", bytearray(str(current_distance)))

                        wait(12, MSEC)
            except Exception as e:
                print(ERROR, "Travel Distance Saving Failed:", e)
                    
    except Exception as e:
        print(ERROR, e)

main()