#region VEXcode Generated Robot Configuration
from vex import *
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
brain_inertial = Inertial()



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
PIDValues = {}


version = "2.0.1"

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
        self.BeamArm.reset_position()
        self.PinArm.reset_position()
        self.DriveMain.reset_position()
        brain_inertial.reset_rotation()
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
            self.PinClaw(False)
            self.BeamClaw(False)
        if Print:
            brain.screen.print(str(self.Time))
        if ProgramStop:
            brain.program_stop()
        if Block:
            while True:
                wait(5000, MSEC)
    def BeamClaw(self, Grab):
        if Grab:
            self.Claws.retract(CYLINDER1)
        else:
            self.Claws.extend(CYLINDER1)
    def PinClaw(self, Grab):
        if Grab:
            self.Claws.retract(CYLINDER2)
        else:
            self.Claws.extend(CYLINDER2)


    def InitPID(self, K, KTurn, Pin, Beam, Drive=(None, None)):
        print(DEBUG, "Initializing PID Controller")
        self.K = K
        self.KTurn = KTurn
        self.KpPin = Pin
        self.KpBeam = Beam
        if Drive:
            self.DefaultLeft, self.DefaultRight = Drive
        DegreesPerWheelRotation = 360 / 2.5
        self.DegreesPerInch = DegreesPerWheelRotation / (2.5 * math.pi)
        print(DEBUG, "PID Initialized")
    def PIDDrive(self, Direction, TargetPos, Reset=True, K=None, RightMotor=None, StationaryWaitTime=120, LeftMotor=None, ID=None, SpeedScale=1, Timeout=999000):
        global OverallScale, PIDDriveScale, PIDValues
        
        SpeedScale *= OverallScale * PIDDriveScale
        TimeoutTimer = Timer()
        StationaryTime = TimeoutTimer.time(MSEC)
        IsStationary = True
        if self.Debug:
            print("\033[32mDrive", Direction, TargetPos, "radians?")
        if not RightMotor:
            RightMotor = self.DefaultRight
        if not LeftMotor:
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
        
        PIDValues[ID] = {"distance": TargetPos}

        while CurrentTime < Timeout:
            Target = PIDValues[ID]["distance"]
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
            if RightDerivative * Target > 0.0:  # TargetPos accounts for going backwards (* -1)
                RightDerivative = 0.0
            if LeftDerivative * Target > 0.0:  # TargetPos accounts for going backwards (* -1)
                LeftDerivative = 0.0

            Difference = LeftError - RightError

            # Calculate power and move the robot
            RightPower = (RightError * Kp) + (RightIntegral * Ki) + (RightDerivative * Kd) + (Difference * Kp / 2)
            LeftPower = (LeftError * Kp) + (LeftIntegral * Ki) + (LeftDerivative * Kd) - (Difference * Kp / 2)
            RightMotor.set_velocity(Min(Clamp(RightPower * SpeedScale)), PERCENT)
            LeftMotor.set_velocity(Min(Clamp(LeftPower * SpeedScale)), PERCENT)
            RightMotor.spin(FORWARD)
            LeftMotor.spin(FORWARD)

            if abs(RightError) < 0.1 and abs(LeftError) < 0.1:
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
                    RightIntegral = 0.0
                    LeftIntegral = 0.0

            wait(1, MSEC)

            CurrentTime = TimeoutTimer.time(MSEC)

        RightMotor.stop()
        LeftMotor.stop()
        del TimeoutTimer
        return
    def PIDTurn(self, Direction, TargetPos, K=None, StationaryWaitTime=120, Reset=True, SpeedScale=1, Timeout=999000):
        global OverallScale
        SpeedScale *= OverallScale
        TimeoutTimer = Timer()
        StationaryTime = TimeoutTimer.time(MSEC)
        IsStationary = False
        RightMotor = self.DefaultRight
        LeftMotor = self.DefaultLeft
        if not K:
            K = self.KTurn
        if Direction == LEFT:
            LeftDir = 1
            RightDir = -1
        elif Direction == RIGHT:
            LeftDir = -1
            RightDir = 1
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
            RightMotor.set_velocity(Clamp(Power * SpeedScale * RightDir), PERCENT)
            LeftMotor.set_velocity(Clamp(Power * SpeedScale * LeftDir), PERCENT)
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
        if x:
            self.x = float(x)
        if y:
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

            self.x = self.x
            self.y = self.y
            
            # Reset LastMotorPos
            LastMotorPos = CurrentMotorPos

            if Sampling:
                self.TrackingLoops += 1

        
    def Reset(self):
        self.x, self.y = self.InitialPos
        self.reset_angle = brain_inertial.heading(TURNS)
        brain_inertial.set_heading(0, DEGREES)

    def ToPoint(self, Point, Direction=FORWARD, StopSmooth=False, SpeedScale=1, TurnScale=1, DriveScale=1, DriveTimeout=999000):
        global PIDDriveScale, PIDValues
        PIDDriveScale = SpeedScale * DriveScale
        TargetX, TargetY = Point
        if self.Debug:
            print("\033[0m - Driving from (", round(self.x, 2), ", ", round(self.y, 2), ") to (", TargetX, ", ", TargetY, ") -", sep="") # Driving from (x, y) to (x, y)
    
        IsDriving = False
        self.RunningOdom = True
        self.DriveThread = None
        LastTurn = Robot.GetTime()
        Distance = math.sqrt((TargetX - self.x) ** 2 + (TargetY - self.y) ** 2)
        if Distance < self.Margin * 0.95:
            self.RunningOdom = False
        while self.RunningOdom:
            angle_to_turn_to = round(math.atan2(TargetY - self.y, TargetX - self.x), 4)
            current_angle = round(brain_inertial.heading(TURNS) * math.pi * 2, 4)
            Distance = math.sqrt((TargetX - self.x) ** 2 + (TargetY - self.y) ** 2)
            if Direction == REVERSE:
                angle_to_turn_to += math.pi
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
            
            if abs(degrees_to_turn) > math.tan(self.Margin / Distance) * 1.3:
                # If the angle is off then stop driving forward and turn
                if self.Debug:
                    print(self.x, self.y)
                    print("\033[32mCurrent Angle:", current_angle, "Angle to Point:", angle_to_turn_to, "- Adjusting Angle (After", round((Robot.GetTime() - LastTurn) / 1000, 2), "Seconds)")
                if self.DriveThread != None:
                    self.DriveThread.stop()
                IsDriving = False
                self.StopDrivingSmooth()
                wait(90, MSEC)
                Robot.PIDTurn(turn_direction, degrees_to_turn, SpeedScale=(SpeedScale * TurnScale * 1.7))
                LastTurn = Robot.GetTime()
                wait(90, MSEC)
            elif abs(degrees_to_turn) > 0.01:
                # Slightly adjust angle while driving
                if degrees_to_turn > 0:
                    Robot.DriveRight.set_velocity(Robot.DriveRight.velocity(PERCENT) * 1.25, PERCENT)
                if degrees_to_turn < 0:
                    Robot.DriveLeft.set_velocity(Robot.DriveLeft.velocity(PERCENT) * 1.25, PERCENT)
                PIDValues["OdomDrive"]["distance"] = Distance

            if not IsDriving:
                # Drive if the robot isn't already doing so

                self.DriveThread = Thread(Robot.PIDDrive, (Direction, Distance, "OdomDrive"))
                IsDriving = True

                # Check for stopping the Odom
                if Distance < (self.Margin * 0.95):
                    self.RunningOdom = False

        if self.DriveThread != None:
            self.DriveThread.stop()
        self.RunningOdom = False
        if StopSmooth:
            self.StopDrivingSmooth()
        print("Finished\033[0m")
    
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
Robot.InitPID((12.85, 0.08, 54.3), (27.0, 0.0155, 114.5), 0.45, 0.7, Drive=(Robot.DriveRight, Robot.DriveLeft))
Odom = InitOdometry(debug=True)

print("\n\033[34m---- Initilization Complete ----\033[0m\n")




def Colors():
    cols = [Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN, Color.BLUE, Color.PURPLE]
    col = 0
    while True:
        col = (col + 1) % len(cols)
        Robot.StartButton.set_color(cols[col])
        wait(400, MSEC)

def Autonomous():
    global OverallScale
    CreateThread = Event()
    ChangingColors = Thread(Colors)
    while not Robot.StartButton.pressing():
        wait(40, MSEC)
    ChangingColors.stop()
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
    TrackingThread = Thread(Odom.TrackLocation)
    

    # ---------------------- Starting Autonomous Code ----------------------

    print(DEBUG, "Autonomous Starting...")
    print("\n\033[34m----- Autonomous -----\033[0m\n")
    
    


    # -- Get First Pins --

    Robot.SpinArm("Beam", 90)


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


    return# brain.program_stop()



def main():
    VidTimer = Timer()
    AutoThread = Thread(Autonomous)
    #brain.buttonCheck.pressed(Autonomous2)


main()
