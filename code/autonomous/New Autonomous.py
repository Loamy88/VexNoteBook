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
# 	Project:      United Robotics 180 Flip Code
#   Team:         26277E
# 	Author:       Euan O'Brien
# 	Description:  VEX IQ python program for advanced autonomous
# 
# ------------------------------------------


print("\n\n\033[34m-====-   Program Start   -====-\033[0m\n")

DEBUG = "\033[35m[DEBUG]\033[0m"
ERROR = "\033[31m[ERROR]\033[0m"


OverallScale = 1.1
PIDDriveScale = 1
PIDValues = {}


version = "2.1.0"

print(DEBUG, "180 Flip Autonomous Code Version:", version)




# Initialize Motors

class Init:
    def __init__(self, Debug=False):
        print(DEBUG, "Initilizing Devices")
        if brain.battery.capacity() <= 75:
            self.LowBat()

        self.StartButton = Touchled(Ports.PORT6)
        self.Debug = Debug
        self.StartButton.set_fade(FadeType.SLOW)
        self.StartButton.on(Color.YELLOW)

        self.Control = Controller()

        self.BeamArm = MotorGroup(Motor(Ports.PORT7), Motor(Ports.PORT1, True))
        self.BeamArm.set_stopping(HOLD)
        self.BeamArm.stop()

        self.Claws = Pneumatic(Ports.PORT11)
        self.Claws.retract(CYLINDER1)
        self.Claws.retract(CYLINDER2)

        self.PinArm = MotorGroup(Motor(Ports.PORT10), Motor(Ports.PORT4, True))
        self.PinArm.set_stopping(HOLD)
        self.PinArm.set_position(0, DEGREES)
        self.PinArm.stop()

        for motor in [self.BeamArm, self.PinArm]:
            motor.set_velocity(100, PERCENT)

        self.DriveLeft = Motor(Ports.PORT9, True)
        self.DriveRight = Motor(Ports.PORT3)
        self.DriveMain = MotorGroup(self.DriveLeft, self.DriveRight)
        self.DriveMain.set_velocity(100, PERCENT)
        self.DriveMain.set_stopping(BRAKE)

        self.BeamArm.reset_position()
        self.PinArm.reset_position()
        self.DriveMain.reset_position()
        brain_inertial.reset_rotation()
        print(DEBUG, "Devices Initilized")
    def LowBat(self):
        batt = brain.battery.capacity()
        note = round(batt / 20)
        brain.play_note(3, note, 500)
        brain.screen.print("Low Battery: " + str(batt))
    def BeamClaw(self, Grab):
        if Grab:
            self.Claws.extend(CYLINDER1)
        else:
            self.Claws.retract(CYLINDER1)
    def PinClaw(self, Grab):
        if Grab:
            self.Claws.extend(CYLINDER2)
        else:
            self.Claws.retract(CYLINDER2)


    def InitPID(self, K, KTurn, Pin, Beam, Drive=(None, None)):
        print(DEBUG, "Initializing PID Controller")
        self.K = K
        self.KTurn = KTurn
        self.KpPin = Pin
        self.KpBeam = Beam
        if Drive:
            self.DefaultRight, self.DefaultLeft = Drive
        DegreesPerWheelRotation = 360 / 2.5
        self.DegreesPerInch = DegreesPerWheelRotation / (2.5 * math.pi)
        print(DEBUG, "PID Initialized")
    def PIDDrive(self, Direction, TargetPos, ID=None, Reset=True, K=None, StationaryWaitTime=120, SpeedScale=1, Timeout=999000):
        global OverallScale, PIDDriveScale, PIDValues
        
        SpeedScale *= OverallScale * PIDDriveScale
        TimeoutTimer = Timer()
        StationaryTime = TimeoutTimer.time(MSEC)
        IsStationary = True
        if self.Debug:
            print("\033[32mDrive", Direction, TargetPos, "radians?")
        RightMotor = self.DefaultRight
        LeftMotor = self.DefaultLeft
        if Reset:
            LeftStart = LeftMotor.position(TURNS) * math.pi * 2
            RightStart = RightMotor.position(TURNS) * math.pi * 2
        if not K:
            K = self.K
        Kp = K[0]
        Ki = K[1]
        Kd = K[2]
        RightIntegral = 0
        LeftIntegral = 0
        RightLastError = 0
        LeftLastError = 0
        if Direction == REVERSE:
            TargetPos *= -1
        
        CurrentTime = TimeoutTimer.time(MSEC)
        
        PIDValues[ID] = {"distance": TargetPos, "direction": Direction, "running": True, "speed": SpeedScale}

        while CurrentTime < Timeout:
            Target = PIDValues[ID]["distance"]
            Speed = PIDValues[ID]["speed"]
            # Calculate error
            RightPos = (RightMotor.position(TURNS) * math.pi * 2) - RightStart
            LeftPos = (LeftMotor.position(TURNS) * math.pi * 2) - LeftStart
            RightError = Target - RightPos
            LeftError = Target - LeftPos

            # Track accumulated error
            RightIntegral += RightError
            LeftIntegral += LeftError
            RightIntegral = Clamp(RightIntegral, 300, -300)
            LeftIntegral = Clamp(LeftIntegral, 300, -300)

            # Calculate derivative
            RightDerivative = RightError - RightLastError
            LeftDerivative = LeftError - LeftLastError
            RightLastError = RightError
            LeftLastError = LeftError
            # Stop going faster if error is increasing
            if RightDerivative * Target > 0.0:  # Target accounts for going backwards (* -1)
                RightDerivative = 0.0
            if LeftDerivative * Target > 0.0:  # Target accounts for going backwards (* -1)
                LeftDerivative = 0.0

            Difference = LeftError - RightError

            # Calculate power and move the robot
            RightPower = (RightError * Kp) + (RightIntegral * Ki) + (RightDerivative * Kd) + (Difference * Kp / 2)
            LeftPower = (LeftError * Kp) + (LeftIntegral * Ki) + (LeftDerivative * Kd) - (Difference * Kp / 2)
            RightMotor.set_velocity(Min(Clamp(RightPower * Speed)), PERCENT)
            LeftMotor.set_velocity(Min(Clamp(LeftPower * Speed)), PERCENT)
            RightMotor.spin(FORWARD)
            LeftMotor.spin(FORWARD)

            if abs(RightError) < 0.1 and abs(LeftError) < 0.1:
                IsStationary = True
                if CurrentTime - StationaryTime > StationaryWaitTime:
                    RightMotor.stop()
                    LeftMotor.stop()
                    del TimeoutTimer
                    PIDValues[ID]["running"] = False
                    return
            else:
                StationaryTime = CurrentTime
                if IsStationary:
                    IsStationary = False
                    RightIntegral = 0.0
                    LeftIntegral = 0.0

            wait(1, MSEC)

            CurrentTime = TimeoutTimer.time(MSEC)

        RightMotor.stop()
        LeftMotor.stop()
        del TimeoutTimer
        PIDValues[ID]["running"] = False
        return
    def PIDTurn(self, Direction, TargetPos, K=None, StationaryWaitTime=120, Reset=True, SpeedScale=1, Timeout=999000):
        global OverallScale
        SpeedScale *= OverallScale
        print("Turn", Direction, TargetPos, "radians")
        TimeoutTimer = Timer()
        StationaryTime = TimeoutTimer.time(MSEC)
        IsStationary = False
        RightMotor = self.DefaultRight
        LeftMotor = self.DefaultLeft
        if not K:
            K = self.KTurn
        if Direction == LEFT:
            LeftDir = -1
            RightDir = 1
        elif Direction == RIGHT:
            LeftDir = 1
            RightDir = -1
        Kp = K[0]
        Ki = K[1]
        Kd = K[2]
        Integral = 0
        LastError = 0
        if Reset:
            Start = brain_inertial.rotation(TURNS) * 2 * math.pi
        CurrentTime = TimeoutTimer.time(MSEC)
        while TimeoutTimer.time(MSEC) < Timeout:
            # Calculate error
            Pos = ((brain_inertial.rotation(TURNS) * 2 * math.pi) - Start) * (-1 if Direction == LEFT else 1)
            Error = TargetPos - Pos
            print(Error)

            # Track accumulated error
            Integral += Error
            Integral = Clamp(Integral, 300, -300)

            # Calculate derivative
            Derivative = Error - LastError
            LastError = Error
            # Stop going faster if error is increasing
            if Derivative * TargetPos > 0.0:  # TargetPos accounts for going backwards (* -1)
                Derivative = 0.0

            # Calculate and apply power
            Power = (Error * Kp) + (Derivative * Kd) + (Integral * Ki)
            RightMotor.set_velocity(Min(Clamp(Power * SpeedScale * RightDir)), PERCENT)
            LeftMotor.set_velocity(Min(Clamp(Power * SpeedScale * LeftDir)), PERCENT)
            RightMotor.spin(FORWARD)
            LeftMotor.spin(FORWARD)

            if Error < 0.08:
                IsStationary = True
                if CurrentTime - StationaryTime > StationaryWaitTime:
                    RightMotor.stop()
                    LeftMotor.stop()
                    del TimeoutTimer
                    return
            else:
                StationaryTime = CurrentTime
                if IsStationary:
                    IsStationary = False
                    Integral = 0.0

            wait(1, MSEC)

            CurrentTime = TimeoutTimer.time(MSEC)

        RightMotor.stop()
        LeftMotor.stop()
        del TimeoutTimer
        return
    def SpinArm(self, Arm, Rotation, Kp=None, Reset=True, Timeout=999000):
        if Arm.lower() == "pin":
            Motor_ = self.PinArm
            GearRatio = (1 / 3)
            if not Kp:
                Kp = self.KpPin
        elif Arm.lower() == "beam":
            Motor_ = self.BeamArm
            GearRatio = (1 / 5)
            if not Kp:
                Kp = self.KpBeam

        TargetAngle = Rotation / GearRatio

        if Reset:
            Motor_.reset_position()

        TimeoutTimer = Timer()

        while TimeoutTimer.time(MSEC) < Timeout:
            Pos = Motor_.position(DEGREES)
            Error = TargetAngle - Pos
            
            Power = Error * Kp
            Motor_.set_velocity(Clamp(Power), PERCENT)
            Motor_.spin(FORWARD)

            if abs(Error) < 1 / GearRatio:
                Motor_.stop()
                del TimeoutTimer
                return
            
        Motor_.stop()
        del TimeoutTimer
        return


class InitOdometry:
    def __init__(self, x=None, y=None, DoReset=True, InitialPos=(0, 0), debug=False, MarginOfError=0.3):
        self.InitialPos = InitialPos
        self.Margin = MarginOfError
        self.RunningOdom = False
        self.Debug = debug
        self.x, self.y = 0.0, 0.0
        if DoReset:
            self.Reset()
        if x != None:
            self.x = float(x)
        if y != None:
            self.y = float(y)

    def ShowData(self):
        self.ShowingData = True

        while self.ShowingData:
            Heading   = "Heading:    " + str(brain_inertial.heading(DEGREES))
            Rotation  = "Rotation:   " + str(brain_inertial.rotation(DEGREES))
            PositionX = "X Position: " + str(self.x)
            PositionY = "Y Position: " + str(self.y)
            Data = ("- Data - ", Heading, Rotation, PositionX, PositionY)

            brain.screen.set_font(FontType.MONO12)

            brain.screen.clear_screen()
            brain.screen.set_cursor(1, 1)

            for Value in Data:
                brain.screen.print(Value)
                brain.screen.next_row()

            brain.screen.render()

            wait(10, MSEC)
        
        self.ShowingData = False


    def TrackLocation(self, Sampling=False):
        """Tracks the movement of the wheels in radians of motor rotation"""
        self.Tracking = True

        LastMotorPos = (Robot.DriveLeft.position(TURNS) + Robot.DriveRight.position(TURNS)) * math.pi

        if Sampling:
            self.TrackingLoops = 0

        while self.Tracking:
            CurrentMotorPos = (Robot.DriveLeft.position(TURNS) + Robot.DriveRight.position(TURNS)) * math.pi

            # Find Movement and Angle
            Change = CurrentMotorPos - LastMotorPos
            CurrentAngle = brain_inertial.heading(TURNS) * math.pi * 2

            # Calculate the Change in x and y
            ChangeX = math.cos(CurrentAngle) * Change
            ChangeY = math.sin(CurrentAngle) * Change

            # Apply Change
            self.x += ChangeX
            self.y += ChangeY

            # Reset LastMotorPos
            LastMotorPos = CurrentMotorPos

            if Sampling:
                self.TrackingLoops += 1

        
    def Reset(self):
        self.x, self.y = self.InitialPos
        brain_inertial.set_heading(0, DEGREES)
        brain_inertial.set_rotation(0, DEGREES)





def Clamp(Num, Max=100, Min=-100):
    return max(min(Num, Max), Min)

def Min(Num, Lim=7.5):
    if Num < 0:
        return min(Num, -Lim)
    else:
        return max(Num, Lim)


print("\n\033[34m---- Initilizing ----\n\033[0m")

Robot = Init(Debug=True)
Robot.InitPID((12.95, 0.093, 20.0), (21.0, 0.0155, 114.5), 0.45, 0.7, Drive=(Robot.DriveRight, Robot.DriveLeft))
Odom = InitOdometry(debug=True)

print("\n\033[34m---- Initilization Complete ----\033[0m\n")


def FixSlack():
    CurrentVelocity = -100
    Robot.DriveRight.spin(FORWARD)
    Robot.DriveLeft.spin(FORWARD)
    while CurrentVelocity < -10:
        CurrentVelocity *= 0.9975
        Robot.DriveRight.set_velocity(CurrentVelocity, PERCENT)
        Robot.DriveLeft.set_velocity(CurrentVelocity, PERCENT)
        wait(1, MSEC)
    Robot.DriveRight.stop()
    Robot.DriveLeft.stop()

def LowerBeam():
    wait(500, MSEC)
    Robot.BeamArm.spin(REVERSE)
    wait(500, MSEC)
    Robot.BeamArm.set_stopping(COAST)
    Robot.BeamArm.stop()
    



def Autonomous():
    global OverallScale
    Robot.StartButton.set_color(Color.BLUE)
    Robot.Claws.pump_on()
    while not Robot.StartButton.pressing():
        wait(40, MSEC)
    Robot.StartButton.set_fade(FadeType.OFF)
    Robot.StartButton.on(Color.RED)
    brain_inertial.set_heading(0, DEGREES)
    Odom.Reset()
    print(DEBUG, "Autonomous Ready, Waiting to Start...")
    while Robot.StartButton.pressing():
        pass
    Robot.StartButton.set_brightness(50)
    Robot.StartButton.set_blink(Color.GREEN, 0.75, 1.25)
    #AutoTimer = Timer()
    TrackingThread = Thread(Odom.TrackLocation)
    

    # ---------------------- Starting Autonomous Code ----------------------

    print(DEBUG, "Autonomous Starting...")
    print("\n\033[34m----- Autonomous -----\033[0m\n")
    
    


    # -- Get First Pins --

    Thread(LowerBeam)
    Robot.PIDDrive(FORWARD, 3.2)
    Robot.PIDTurn(LEFT, 0.17)
    Robot.PIDDrive(FORWARD, 5.0)
    Robot.PIDTurn(RIGHT, 0.68)
    Robot.PIDDrive(FORWARD, 3.4)
    Robot.PIDTurn(RIGHT, 0.19)
    Robot.PIDDrive(FORWARD, 4.8, SpeedScale=0.86)
    Robot.PinClaw(True)


    # -- Get Second Pins --

    Robot.SpinArm("pin", 35)
    Robot.PIDDrive(FORWARD, 1.85)
    Robot.PIDTurn(RIGHT, 0.97)
    Robot.PIDDrive(FORWARD, 4.25)
    Robot.PIDTurn(RIGHT, 0.2)
    Robot.PIDDrive(FORWARD, 2.8)


    # -- Stack --

    Robot.SpinArm("pin", -35, Timeout=500)
    Robot.PinClaw(False)
    Robot.SpinArm("pin", -35, Timeout=300)
    Robot.PinClaw(True)
    Robot.SpinArm("pin", 30)


    # -- Get Beam --

    Robot.PIDTurn(RIGHT, 1.17)
    Robot.PIDDrive(REVERSE, 2.72, Timeout=600) # should drive 2.42
    Robot.BeamClaw(True)


    # -- Flip Pins --

    Robot.SpinArm("pin", 180, Timeout=1400)
    Robot.PinClaw(False)
    Robot.SpinArm("pin", 200, Timeout=1400)


    # -- Put Pin on Standoff --

    Robot.BeamArm.set_stopping(HOLD)
    Robot.PIDDrive(FORWARD, 1.08)
    Robot.SpinArm("beam", 100, Timeout=1300)
    # Fix Position
    Robot.PIDTurn(RIGHT, 0.1)
    Robot.PIDDrive(FORWARD, 5.0, Timeout=1700) # should drive 4.28
    Robot.PIDDrive(REVERSE, 2.45)


    # -- Put Beam on Standoff Pin --

    Robot.PIDTurn(RIGHT, 2.9)
    Robot.PIDDrive(REVERSE, 2.45)
    Robot.BeamArm.set_velocity(40, PERCENT)
    Robot.BeamArm.spin(REVERSE)
    wait(100, MSEC)
    Robot.BeamClaw(False)
    Robot.BeamArm.stop()
    Robot.SpinArm("beam", 20, Timeout=100)
    Robot.PIDDrive(FORWARD, 3.2)


    brain.play_note(3,0,400)
    return# brain.program_stop()



def main():
    Robot.BeamArm.set_stopping(HOLD)

    # calibrate
    brain_inertial.calibrate()
    print(DEBUG, "Calibrated")
    wait(100, MSEC)

    # wait for first press to start
    while not Robot.StartButton.pressing():
        wait(10, MSEC)
    print(DEBUG, "Button Pressed...")

    # fix slack
    wait(100, MSEC)
    print(DEBUG, "Spinning Motors to Fix Slack")
    FixSlack()

    while Robot.StartButton.pressing():
        wait(10, MSEC)

    # run auton
    AutoThread = Thread(Autonomous)


main()
