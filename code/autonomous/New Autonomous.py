#region VEXcode Generated Robot Configuration
from vex import *
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
brain_inertial = Inertial()


'''
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
'''

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      VEXcode Project
# 	Author:       VEX
# 	Created:
# 	Description:  VEXcode IQ Python Project
# 
# ------------------------------------------


print("\n\n\033[34m-====-   Program Start   -====-\033[0m\n")

DEBUG = "\033[35m[DEBUG]\033[0m"
ERROR = "\033[31m[ERROR]\033[0m"


print(DEBUG, "Starting...")


OverallScale = 1.1
PIDDriveScale = 1
PIDStopper = {}


version = "2.0.0"

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
        self.StartButton.on(Color.RED)
        self.Control = Controller()
        self.BeamArm = MotorGroup(Motor(Ports.PORT7), Motor(Ports.PORT1, True))
        self.Claws = Pneumatic(Ports.PORT11)
        self.Claws.pump_on()
        self.Claws.retract(CYLINDER1)
        self.Claws.retract(CYLINDER2)
        self.PinArm = MotorGroup(Motor(Ports.PORT10), Motor(Ports.PORT4, True))
        self.PinArm.set_stopping(HOLD)
        self.PinArm.set_position(0, DEGREES)
        for motor in [self.BeamArm, self.PinArm]:
            motor.set_velocity(100, PERCENT)
        self.DriveLeft = Motor(Ports.PORT9, True)
        self.DriveRight = Motor(Ports.PORT3)
        self.DriveMain = MotorGroup(self.DriveLeft, self.DriveRight)
        self.DriveMain.set_velocity(100, PERCENT)
        self.DriveMain.set_stopping(BRAKE)
        self.Reset()
        print(DEBUG, "Devices Initilized")
    def LowBat(self):
        bat = brain.battery.capacity()
        if bat < 60:
            color = "\033[31m"
        else:
            color = "\033[91m"
        print(DEBUG, "Low Battery:", color, bat)
        Percent = bat / 100
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
    def Start(self):
        self.AutoTimer = Timer()
    def GetTime(self):
        self.Time = self.AutoTimer.time()
        return self.Time
    def Stop(self, Print=True, Clear=True, ProgramStop=False, Block=True, OpenClaws=True):
        self.GetTime()
        if Clear:
            brain.screen.clear_screen()
        if OpenClaws:
            self.PinRelease()
            self.BeamRelease()
        if Print:
            brain.screen.print(str(self.Time))
        if ProgramStop:
            brain.program_stop()
        if Block:
            while True:
                wait(5000, MSEC)
    def Reset(self):
        self.BeamArm.reset_position()
        self.PinArm.reset_position()
        self.DriveMain.reset_position()
        brain_inertial.reset_rotation()
    def BeamRelease(self):
        self.Claws.retract(CYLINDER1)
    def BeamGrab(self):
        self.Claws.extend(CYLINDER1)
    def PinRelease(self):
        self.Claws.retract(CYLINDER2)
    def PinGrab(self):
        self.Claws.extend(CYLINDER2)


    def InitPID(self, Kp, Ki, Kd, Drive=(None, None)):
        print(DEBUG, "Initializing PID Controller")
        self.K = [Kp, Ki, Kd]
        if Drive:
            self.DefaultLeft, self.DefaultRight = Drive
        DegreesPerWheelRotation = 360 / GearRatio
        self.DegreesPerInch = DegreesPerWheelRotation / (WheelDiameter * math.pi)
        print(DEBUG, "PID Initialized")
    def PIDDrive(self, Direction, TargetPos, K=None, RightMotor=None, LeftMotor=None, Reset=True, SpeedScale=1, Timeout=999000):
        global OverallScale, PIDDriveScale
        SpeedScale *= OverallScale * PIDDriveScale
        TimeoutTimer = Timer()
        if self.Debug:
            print("\033[32mDrive", Direction, TargetDistance, "inches")
        if not RightMotor:
            RightMotor = self.DefaultRight
        if not LeftMotor:
            LeftMotor = self.DefaultLeft
        if Reset:
            LeftStart = LeftMotor.position(Unit)
            RightStart = RightMotor.position(Unit)
        if not K:
            K = self.K
        Kp = K[0]
        Ki = K[1]
        Kd = K[2]
        RightIntegral = 0
        LeftIntegral = 0
        RightLastError = 0
        LeftLastError = 0
        #f = open("logs.txt", "w")
        while TimeoutTimer.time(MSEC) < Timeout:
            RightPos = (RightMotor.position(Unit) - RightStart) * (-1 if Direction == REVERSE else 1)
            LeftPos = (LeftMotor.position(Unit) - LeftStart) * (-1 if Direction == REVERSE else 1)
            RightError = TargetPos - RightPos
            LeftError = TargetPos - LeftPos
            RightIntegral += RightError
            LeftIntegral += LeftError
            RightIntegral = Clamp(RightIntegral, 300, -300)
            LeftIntegral = Clamp(LeftIntegral, 300, -300)
            RightDerivative = RightError - RightLastError
            LeftDerivative = LeftError - LeftLastError
            RightLastError = RightError
            LeftLastError = LeftError
            Difference = LeftError - RightError

            RightPower = (RightError * Kp) + (RightDerivative * Kd) + (RightIntegral * Ki) + (Difference * Kp / 2)
            LeftPower = (LeftError * Kp) + (LeftDerivative * Kd) + (LeftIntegral * Ki) - (Difference * Kp / 4)
            RightMotor.set_velocity(Min(Clamp(RightPower * SpeedScale)), PERCENT)
            LeftMotor.set_velocity(Min(Clamp(LeftPower * SpeedScale)), PERCENT)
            RightMotor.spin(Direction)
            LeftMotor.spin(Direction)
            #f.write("r: " + str(RightError) + str(RightPower) + "l: " + str(LeftError) + str(LeftPower) + "\n")

            if RightError < 3 and LeftError < 3:
                RightMotor.stop()
                LeftMotor.stop()
                #f.write("Done\n")
                del TimeoutTimer
                return

            wait(1, MSEC)

        RightMotor.stop()
        LeftMotor.stop()
        del TimeoutTimer
        return
    def PIDTurn(self, Direction, TargetPos, K=None, RightMotor=None, LeftMotor=None, Reset=True, SpeedScale=1, Timeout=999000):
        global OverallScale
        SpeedScale *= OverallScale
        TimeoutTimer = Timer()
        if self.Debug:
            print("\033[32mTurn", Direction, TargetPos, "Radians", end="  ")
        if not RightMotor:
            RightMotor = self.DefaultRight
        if not LeftMotor:
            LeftMotor = self.DefaultLeft
        if not K:
            K = self.K
        if Direction == LEFT:
            Left = FORWARD
            Right = REVERSE
        elif Direction == RIGHT:
            Left = REVERSE
            Right = FORWARD
        Kp = K[0]
        Ki = K[1]
        Kd = K[2]
        Integral = 0
        LastError = 0
        if Reset:
            Start = brain_inertial.rotation(TURNS) * 2 * math.pi
        while TimeoutTimer.time(MSEC) < Timeout:
            Pos = ((brain_inertial.rotation(TURNS) * 2 * math.pi) - Start) * (-1 if Direction == LEFT else 1)
            Error = TargetPos - Pos
            Integral += Error
            Integral = Integral
            Derivative = Error - LastError
            LastError = Error
            print(TimeoutTimer.time(MSEC), "-", Error, end="  ")

            Power = (Error * Kp) + (Derivative * Kd) + (Integral * Ki)
            RightMotor.set_velocity(Clamp(Power * SpeedScale), PERCENT)
            LeftMotor.set_velocity(Clamp(Power * SpeedScale), PERCENT)
            RightMotor.spin(Right)
            LeftMotor.spin(Left)

            if Error < 1:
                RightMotor.stop()
                LeftMotor.stop()
                del TimeoutTimer
                return

            wait(1, MSEC)

        RightMotor.stop()
        LeftMotor.stop()
        del TimeoutTimer
        return
    def SpinPinArm(self, Rotation, GearRatio=(1 / 3), Kp=3, Reset=True, Timeout=999000):
        Motor_ = Robot.PinArm
        TargetAngle = Rotation * GearRatio

        if Reset:
            Motor_.reset_position()

        TimeoutTimer = Timer()

        while TimeoutTimer.time(MSEC) < Timeout:
            Pos = Motor_.position(DEGREES)
            Error = TargetAngle - Pos
            
            Power = Error * Kp
            Motor_.set_velocity(Clamp(Power), PERCENT)
            Motor_.spin(FORWARD)
            
        Motor_.stop()
        del TimeoutTimer
        return


class InitOdometry:
    def __init__(self, x=None, y=None, Units=INCHES, DoReset=True, InitialPos=(0, 0), debug=False, MarginOfError=0.5):
        self.Units = Units
        self.InitialPos = InitialPos
        self.Margin = MarginOfError
        self.RunningOdom = False
        self.Debug = debug
        self.x, self.y = 0.0, 0.0
        if DoReset:
            self.Reset()
        if x:
            self.x = float(x)
        if y:
            self.y = float(y)
        
    def RadiansToInches(self, rad):
        degrees = rad * (180 / math.pi)
        return degrees / Robot.DegreesPerInch

    def close(self):
        self.Tracking = False
        self.RunningOdom = False
        self.ShowingData = False

    def ShowData(self):
        self.ShowingData = True

        while self.ShowingData:
            Heading   = "Heading:    " + str(brain_inertial.heading(DEGREES))
            Rotation  = "Rotation:   " + str(brain_intertial.rotation(DEGREES))
            PositionX = "X Position: " + str(self.x)
            PositionY = "Y Position: " + str(self.y)
            Data = (Heading, Rotation, PositionX, PositionY)

            brain.screen.set_font(FontType.MONO12)

            brain.screen.clear_screen()

            for Value in Data:
                brain.screen.print(Value)
                brain.screen.next_row()

            brain.screen.render()

            wait(10, MSEC)


    def TrackLocationInterial(self):
        """Tracks the movement of the brains inertial sensor in g-seconds"""
        self.Tracking = True

        TrackingTimer = Timer()

        while self.Tracking:
            DeltaTime = TrackingTimer.time(MSEC)
            TrackingTimer.clear()

            # Find acceleration
            AccelX = brain_inertial.acceleration(XAXIS)
            AccelY = brain_inertial.acceleration(YAXIS)

            # Calculate the local change in g-seconds
            DeltaLocalForward = (-AccelY * DeltaTime) / 1000
            DeltaLocalRight = (AccelX * DeltaTime) / 1000

            CurrentAngle = brain_inertial.heading(TURNS) * math.pi * 2
            CurrentAngleRight = CurrentAngle + (math.pi / 2)

            DeltaX = (math.cos(CurrentAngle) * DeltaLocalForward) + (math.cos(CurrentAngleRight) * DeltaLocalRight)
            DeltaY = (math.sin(CurrentAngle) * DeltaLocalForward) + (math.sin(CurrentAngleRight) * DeltaLocalRight)

            self.x += DeltaX
            self.y += DeltaY




    def TrackLocation(self, Sampling=False):
        """Tracks the movement of the wheels in radians of motor rotation"""
        self.Tracking = True

        LastMotorPos = (Robot.DriveLeft.position(TURNS) + Robot.DriveRight.position(TURNS)) * math.pi

        if Sampling:
            self.TrackingLoops = 0

        while self.Tracking:
            CurrentMotorPos = (Robot.DriveLeft.position(TURNS) + Robot.DriveRight.position(TURNS)) * math.pi

            # Find Movement and Angle
            Change = self.RadiansToInches(CurrentMotorPos - LastMotorPos)
            CurrentAngle = brain_inertial.heading(TURNS) * math.pi * 2

            # Calculate the Change in x and y
            ChangeX = math.cos(CurrentAngle) * Change
            ChangeY = math.sin(CurrentAngle) * Change

            # Apply Change
            self.x += ChangeX
            self.y += ChangeY

            self.x = self.x, 4
            self.y = self.y, 4
            
            # Reset LastMotorPos
            LastMotorPos = CurrentMotorPos

            if Sampling:
                self.TrackingLoops += 1

        
    def Reset(self):
        self.x, self.y = self.InitialPos
        self.reset_angle = brain_inertial.heading(TURNS)
        brain_inertial.set_heading(0, DEGREES)

    def ToPoint(self, Point, Direction=FORWARD, StopSmooth=True, SpeedScale=1, TurnScale=1, DriveScale=1, DriveTimeout=999000):
        global PIDDriveScale
        PIDDriveScale = SpeedScale * DriveScale
        TargetX, TargetY = Point
        if self.Debug:
            print("\033[0m - Driving from (", round(self.x, 2), ", ", round(self.y, 2), ") to (", TargetX, ", ", TargetY, ") -", sep="") # Driving from (x, y) to (x, y)
    
        IsDriving = False
        self.RunningOdom = True
        self.DriveThread = None
        LastTurn = Robot.GetTime()
        while self.RunningOdom:
            angle_to_turn_to = round(math.atan2(TargetY - self.y, TargetX - self.x), 4)
            current_angle = round(brain_inertial.heading(TURNS) * math.pi * 2, 4)
            if Direction == REVERSE:
                angle_to_turn_to += math.pi
            # - Turn -
            degrees_to_turn = round((angle_to_turn_to - current_angle) % (2 * math.pi), 4)

            # Get optimal turn direction
            if degrees_to_turn > math.pi:
                degrees_to_turn -= 2 * math.pi
            if degrees_to_turn < -math.pi:
                degrees_to_turn += 2 * math.pi
            if degrees_to_turn < 0:
                turn_direction = LEFT
                degrees_to_turn *= -1
            else:
                turn_direction = RIGHT

            degrees_to_turn = round(degrees_to_turn, 4)
            
            if abs(degrees_to_turn) > math.tan(self.Margin / math.sqrt((TargetX - self.x) ** 2 + (TargetY - self.y) ** 2)) * 1.3:
                # If the angle is off then stop driving forward and turn
                if self.Debug:
                    print(self.x, self.y)
                    print("\033[32mCurrent Angle:", current_angle, "Angle to Point:", angle_to_turn_to, "- Adjusting Angle (After", round((Robot.GetTime() - LastTurn) / 1000, 2), "Seconds)")
                if self.DriveThread != None:
                    self.DriveThread.stop()
                IsDriving = False
                self.StopDrivingSmooth()
                wait(90, MSEC)
                Robot.PIDTurn(turn_direction, degrees_to_turn, SpeedScale=(SpeedScale * TurnScale * 1.7), Radians=True)
                LastTurn = Robot.GetTime()
                wait(90, MSEC)

            if not IsDriving:
                # Drive if the robot isn't already doing so
                Distance = round(math.sqrt((TargetX - self.x) ** 2 + (TargetY - self.y) ** 2), 4)

                self.DriveThread = Thread(Robot.PIDDrive, (Direction, Distance))
                IsDriving = True

                # Check for stopping the Odom
                if math.sqrt((TargetX - self.x) ** 2 + (TargetY - self.y) ** 2) < (self.Margin * 0.95):
                    self.RunningOdom = False

        self.DriveThread.stop()
        self.RunningOdom = False
        if StopSmooth:
            self.StopDrivingSmooth()
        print("\033[0m")
    
    def StopDrivingSmooth(self, Rate=0.994):
        CurrentVelocity = 100
        while CurrentVelocity > 10:
            CurrentVelocity = (Robot.DriveLeft.velocity(PERCENT) + Robot.DriveRight.velocity(PERCENT)) / 2
            Robot.DriveRight.set_velocity(CurrentVelocity * Rate, PERCENT)
            Robot.DriveLeft.set_velocity(CurrentVelocity * Rate, PERCENT)



def Clamp(Num, Max=100, Min=-100):
    return max(min(Num, Max), Min)

def Min(Num, Lim=7.5):
    if Num < 0:
        return min(Num, -Lim)
    else:
        return max(Num, Lim)


print("\n\033[34m---- Initilizing ----\n\033[0m")

Robot = Init(Debug=True)
Robot.InitPID(0.34, 0.002, 0.55, 2.5, 2.5, Drive=(Robot.DriveRight, Robot.DriveLeft))
Odom = InitOdometry(debug=True)

print("\n\033[34m---- Initilization Complete ----\033[0m\n")





def Autonomous():
    global OverallScale, PIDStopper
    CreateThread = Event()
    cols = [Color.RED, Color.ORANGE, Color.ELLOW, Color.GREEN, Color.BLUE, Color.PURPLE]
    col = 0
    while not Robot.StartButton.pressing():
        col = (col + 1) % len(cols)
        Robot.StartButton.set_color(cols[col])
        wait(400, MSEC)
    Robot.StartButton.set_fade(FadeType.OFF)
    Robot.StartButton.on(Color.RED)
    brain_inertial.set_heading(0, DEGREES)
    Odom.Reset()
    print(DEBUG, "Autonomous Ready, Waiting to Start...")
    while Robot.StartButton.pressing():
        pass
    Robot.StartButton.set_brightness(50)
    Robot.StartButton.set_blink(Color.GREEN, 0.75, 1.25)
    Robot.Start()
    TrackingThread = Thread(Odom.TrackLocationInterial)
    

    # ---------------------- Starting Autonomous Code ----------------------

    print(DEBUG, "Autonomous Starting...")
    print("\n\033[34m----- Autonomous -----\033[0m\n")
    
    


    # -- Get First Pins --

    Odom.ShowData()


    # -- Get Second Pins --

    # Placeholder


    # -- Stack --

    # Placeholder


    # -- Get Beam --

    # Placeholder


    # -- Flip Pins --

    # Placeholder


    # -- Put Pin on Standoff --

    # Placeholder


    # -- Put Beam on Standoff Pin --

    # Placeholder


    return



def main():
    VidTimer = Timer()
    AutoThread = Thread(Autonomous)
    #brain.buttonCheck.pressed(Autonomous2)


main()
