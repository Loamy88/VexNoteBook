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


print("\033[0m[DEBUG] Starting...")


OverallScale = 1.13
PIDStopper = {}


version = "2.0.0"

print("180 Flip Autonomous Code Version:\033[35m", version, "\033[0m")

DEBUG = "\033[31m[DEBUG]\033[0m"


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
        if self.Debug:
            print("Release Beam")
    def BeamGrab(self):
        self.Claws.extend(CYLINDER1)
        if self.Debug:
            print("Grab Beam")
    def PinRelease(self):
        self.Claws.retract(CYLINDER2)
        if self.Debug:
            print("Release Pin")
    def PinGrab(self):
        self.Claws.extend(CYLINDER2)
        if self.Debug:
            print("Grab Pin")


    def InitPID(self, kP, kI, kD, WheelDiameter, GearRatio, Drive=(None, None)):
        print(DEBUG, "Initializing PID Controller")
        self.k = [kP, kI, kD]
        if Drive:
            self.DefaultLeft, self.DefaultRight = Drive
        DegreesPerWheelRotation = 360 / GearRatio
        self.DegreesPerInch = DegreesPerWheelRotation / (WheelDiameter * math.pi)
        print(DEBUG, "PID Initialized")
    def PIDDrive(self, Direction, TargetDistance, StopID=None, k=None, RightMotor=None, LeftMotor=None, Unit=DEGREES, Reset=True, SpeedScale=1, Timeout=999000):
        global OverallScale, PIDStopper
        if StopID:
            PIDStopper[StopID] = False
        SpeedScale *= OverallScale
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
        TargetPos = self.DegreesPerInch * TargetDistance
        if not k:
            k = self.k
        kP = k[0]
        kI = k[1]
        kD = k[2]
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

            RightPower = (RightError * kP) + (RightDerivative * kD) + (RightIntegral * kI) + (Difference * kP / 2)
            LeftPower = (LeftError * kP) + (LeftDerivative * kD) + (LeftIntegral * kI) - (Difference * kP / 4)
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

            if StopID:
                if PIDStopper[StopID]:
                    RightMotor.stop()
                    LeftMotor.stop()
                    del TimeoutTimer
                    return
            wait(1, MSEC)

        RightMotor.stop()
        LeftMotor.stop()
        del TimeoutTimer
        return
    def PIDTurn(self, Direction, TargetPos, StopID=None, k=None, RightMotor=None, LeftMotor=None, Reset=True, SpeedScale=1, Timeout=999000, Radians=False):
        global OverallScale, PIDStopper
        if StopID:
            PIDStopper[StopID] = False
        SpeedScale *= OverallScale
        if Radians:
            TargetPos *= 180 / math.pi
        TimeoutTimer = Timer()
        if self.Debug:
            print("\033[32mTurn", Direction, TargetPos, "Degrees", end="  Running  ")
        if not RightMotor:
            RightMotor = self.DefaultRight
        if not LeftMotor:
            LeftMotor = self.DefaultLeft
        if Reset:
            brain_inertial.reset_rotation()
        if not k:
            k = self.k
        if Direction == LEFT:
            Left = FORWARD
            Right = REVERSE
        elif Direction == RIGHT:
            Left = REVERSE
            Right = FORWARD
        kP = k[0]
        kI = k[1]
        kD = k[2]
        Integral = 0
        LastError = 0
        while TimeoutTimer.time(MSEC) < Timeout:
            Pos = brain_inertial.rotation() * (-1 if Direction == LEFT else 1)
            Error = TargetPos - Pos
            Integral += Error
            Integral = Clamp(Integral, 300, -300)
            Derivative = Error - LastError
            LastError = Error

            Power = (Error * kP) + (Derivative * kD) + (Integral * kI)
            RightMotor.set_velocity(Min(Clamp(Power * SpeedScale), Lim=10.5), PERCENT)
            LeftMotor.set_velocity(Min(Clamp(Power * SpeedScale), Lim=10.5), PERCENT)
            RightMotor.spin(Right)
            LeftMotor.spin(Left)

            if Error < 3:
                RightMotor.stop()
                LeftMotor.stop()
                del TimeoutTimer
                return

            if StopID:
                if PIDStopper[StopID]:
                    RightMotor.stop()
                    LeftMotor.stop()
                    del TimeoutTimer
                    return
            wait(1, MSEC)

        RightMotor.stop()
        LeftMotor.stop()
        del TimeoutTimer
        if self.Debug:
            print("Finished")
        return
    def PIDSpinMotor(self, Motor_, Direction, Distance_, StopID=None, k=None, GearRatio=1, Unit=DEGREES, Reset=True, SpeedScale=1, Timeout=999000):
        global OverallScale, PIDStopper
        if StopID:
            PIDStopper[StopID] = False
        SpeedScale *= OverallScale
        if self.Debug:
            print("\033[32mSpin", Motor_, Direction, Distance_, "Degrees")
        TimeoutTimer = Timer()
        if Reset:
            Start = Motor_.position(Unit)
        if not k:
            k = self.k
        kP = k[0]
        kI = k[1]
        kD = k[2]
        Integral = 0
        LastError = 0
        while TimeoutTimer.time(MSEC) < Timeout:
            Pos = (Motor_.position(Unit) - Start) * GearRatio
            Error = Distance_ - Pos
            Integral += Error
            Integral = Clamp(Integral, 300, -300)
            Derivative = Error - LastError
            LastError = Error

            Power = (Error * kP) + (Derivative * kD) + (Integral * kI)
            Motor_.set_velocity(Min(Clamp(Power * SpeedScale)), PERCENT)
            Motor_.spin(Direction)

            if Error < 5:
                Motor_.stop()
                del TimeoutTimer
                return

            if StopID:
                if PIDStopper[StopID]:
                    Motor_.stop()
                    del TimeoutTimer
                    return
            wait(1, MSEC)

        Motor_.stop()
        del TimeoutTimer
        return


class InitPTP:
    def __init__(self, x=None, y=None, Units=INCHES, DoReset=True, InitialPos=(0, 0), debug=False):
        self.Units = Units
        self.InertialObject = Inertial()
        self.InitialPos = InitialPos
        self.RunningPTP = False
        self.Debug = debug
        self.x, self.y = 0.0, 0.0
        if DoReset:
            self.Reset()
        if x:
            self.x = float(x)
        if y:
            self.y = float(y)
        if self.Debug:
            TrackingThread = Thread(self.TrackLocation, (True,))
            wait(100, MSEC)
            self.Tracking = False
            if self.Debug:
                print(DEBUG, "Tracking Loops Per Second: ", self.TrackingLoops * 10)
        
    def RadiansToInches(self, rad):
        degrees = rad * (180 / math.pi)
        return degrees / Robot.DegreesPerInch

    def close(self):
        self.Tracking = False
        self.RunningPTP = False

    def TrackLocation(self, Sampling=False):
        self.Tracking = True

        LastMotorPos = (Robot.DriveLeft.position(TURNS) + Robot.DriveRight.position(TURNS)) * math.pi

        if Sampling:
            self.TrackingLoops = 0

        while self.Tracking:
            CurrentMotorPos = (Robot.DriveLeft.position(TURNS) + Robot.DriveRight.position(TURNS)) * math.pi

            # Find Movement and Angle
            Change = self.RadiansToInches(CurrentMotorPos - LastMotorPos)
            CurrentAngle = self.InertialObject.heading(TURNS) * math.pi

            # Calculate the Change in x and y
            ChangeX = math.cos(CurrentAngle) * Change
            ChangeY = math.sin(CurrentAngle) * Change

            # Apply Change
            self.x += ChangeX
            self.y += ChangeY

            self.x = round(self.x, 4)
            self.y = round(self.y, 4)
            
            # Reset LastMotorPos
            LastMotorPos = CurrentMotorPos

            if Sampling:
                self.TrackingLoops += 1

        
    def Reset(self):
        self.x, self.y = self.InitialPos
        self.InertialObject.reset_heading()

    def ToPoint(self, Point, Direction=FORWARD, SpeedScale=1, TurnScale=1.7, DriveScale=1, DriveTimeout=999000):
        x_loc, y_loc = Point
        if self.Debug:
            print("\033[0m - Driving from (", self.x, ", ", self.y, ") to (", x_loc, ", ", y_loc, ") -", sep="") # Driving from (x, y) to (x, y)

            angle_to_turn_to = math.atan2(y_loc - self.y, x_loc - self.x)
            if Direction == REVERSE:
                angle_to_turn_to += math.pi

            current_angle = self.InertialObject.heading(TURNS) * math.pi

            print("\033[32mCurrent Angle:", current_angle, "Angle to Point:", angle_to_turn_to)
    
        IsDriving = False
        self.RunningPTP = True
        self.DriveThread = None
        while self.RunningPTP:
            angle_to_turn_to = round(math.atan2(y_loc - self.y, x_loc - self.x), 4)
            current_angle = round(self.InertialObject.heading(TURNS) * math.pi, 4)
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
                turn_angle = LEFT
                degrees_to_turn *= -1
            else:
                turn_angle = RIGHT

            degrees_to_turn = round(degrees_to_turn, 4)
            
            if abs(degrees_to_turn) > 0.15 and math.sqrt((x_loc - self.x) ** 2 + (y_loc - self.y) ** 2) > 0.5:
                # If the angle is off then stop driving forward and turn
                if self.Debug:
                    print("\033[32mCurrent Angle:", current_angle, "Angle to Point:", angle_to_turn_to, "- Adjusting Angle")
                if self.DriveThread != None:
                    self.DriveThread.stop()
                IsDriving = False
                self.StopDrivingSmooth()
                Robot.PIDTurn(turn_angle, degrees_to_turn, SpeedScale=(SpeedScale * TurnScale), Radians=True)
                wait(50, MSEC)

            if not IsDriving:
                # Drive if the robot isn't already doing so
                Distance = round(math.sqrt((x_loc - self.x) ** 2 + (y_loc - self.y) ** 2), 4)

                self.DriveThread = Thread(Robot.PIDDrive, (Direction, Distance))
                IsDriving = True

                # Check for stopping the PTP
                if math.sqrt((x_loc - self.x) ** 2 + (y_loc - self.y) ** 2) < 0.3:
                    self.RunningPTP = False

        self.DriveThread.stop()
        self.RunningPTP = False
        print("\033[0m")
    
    def StopDrivingSmooth(self, Rate=0.99):
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
PTP = InitPTP(debug=True)

print("\n\033[34m---- Initilization Complete ----\033[0m\n")





def Autonomous():
    global OverallScale, PIDStopper
    CreateThread = Event()
    cols = [Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN, Color.BLUE, Color.PURPLE]
    while not Robot.StartButton.pressing():
        Robot.StartButton.set_color(cols[urandom.randint(1, len(cols)) - 1])
        wait(500, MSEC)
    Robot.StartButton.set_fade(FadeType.OFF)
    Robot.StartButton.on(Color.RED)
    PTP.Reset()
    while Robot.StartButton.pressing():
        pass
    Robot.StartButton.set_brightness(50)
    Robot.StartButton.set_blink(Color.GREEN, 0.75, 1.25)
    Robot.Start()
    TrackingThread = Thread(PTP.TrackLocation)
    

    # ---------------------- Starting Autonomous Code ----------------------

    print(DEBUG, "Autonomous Starting...")
    print("\n\033[34m----- Autonomous -----\033[0m\n")
    
    


    # -- Get First Pins --

    PTP.ToPoint((5, 5))


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
