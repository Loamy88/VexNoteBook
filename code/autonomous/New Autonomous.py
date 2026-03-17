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

OverallScale = 1.13
PIDStopper = {}


version = "1.0.0"

print("180 Flip Autonomous Code Version:", version)




# Initialize Motors

class Init:
    def __init__(self):
        print("[DEBUG] Initilizing...")
        if brain.battery.capacity() <= 75:
            self.LowBat()
        self.StartButton = Touchled(Ports.PORT6)
        self.StartButton.set_fade(FadeType.SLOW)
        self.StartButton.on(Color.RED)
        self.Dist = Distance(Ports.PORT12)
        self.Control = Controller()
        self.BeamArm = MotorGroup(Motor(Ports.PORT7), Motor(Ports.PORT1, True))
        self.Claws = Pneumatic(Ports.PORT11)
        self.Claws.pump_on()
        self.Claws.retract(CYLINDER1)
        self.Claws.retract(CYLINDER2)
        self.PinArm = MotorGroup(Motor(Ports.PORT10), Motor(Ports.PORT4, True))
        self.PinArm.set_stopping(HOLD)
        self.PinArm.set_position(0, DEGREES)
        #  - Simplicity for controlling the claws -
        self.Beam = self.InitBeamClaw(self.Claws)
        self.Pin = self.InitPinClaw(self.Claws)
        for motor in [self.BeamArm, self.PinArm]:
            motor.set_velocity(100, PERCENT)
        self.DriveMotors = self.InitDrive()
        self.DriveMotors.Main.set_stopping(BRAKE)
        print("[DEBUG] Initilized")
    def LowBat(self):
        print("Low Battery:", brain.battery.capacity())
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
            self.Pin.Release()
            self.Beam.Release()
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
        self.DriveMotors.Main.reset_position()
        brain_inertial.reset_rotation()
    def CheckDistance(self, Distance2Check, Error=7, Units=MM):
        return abs(self.Dist.object_distance(Units) - Distance2Check) < Error
    def StopSpinning(self):
        global PIDStopper
        ID = "SpinningFunction"
        Distance2Check = 310

        count = 0
        while True:
            if self.CheckDistance(Distance2Check):
                count += 1
                if count == 26:
                    PIDStopper[ID] = True
                    return
            else:
                count = 0

            if PIDStopper[ID]:
                return

    class InitBeamClaw:
        def __init__(self, Pneumatic_):
            self.Pneu = Pneumatic_
            self.Cylinder = CYLINDER1
        def Release(self):
            self.Pneu.retract(self.Cylinder)
            print("Release Beam")
        def Grab(self):
            self.Pneu.extend(self.Cylinder)
            print("Grab Beam")
    class InitPinClaw:
        def __init__(self, Pneumatic_):
            self.Pneu = Pneumatic_
            self.Cylinder = CYLINDER2
        def Release(self):
            self.Pneu.retract(self.Cylinder)
            print("Release Pin")
        def Grab(self):
            self.Pneu.extend(self.Cylinder)
            print("Grab Pin")
    class InitDrive:
        def __init__(self):
            self.Left = Motor(Ports.PORT9, True)
            self.Right = Motor(Ports.PORT3)
            self.Main = MotorGroup(self.Left, self.Right)
            self.Main.set_velocity(100, PERCENT)
    class InitPID:
        def __init__(self, kP, kI, kD, WheelDiameter, GearRatio, Drive=(None, None)):
            print("[DEBUG] Initializing PID Controller...  ", end="")
            self.k = [kP, kI, kD]
            if Drive:
                self.DefaultLeft, self.DefaultRight = Drive
            DegreesPerWheelRotation = 360 / GearRatio
            self.DegreesPerInch = DegreesPerWheelRotation / (WheelDiameter * math.pi)
            print("  PID Initialized")
        def Drive(self, Direction, TargetDistance, StopID=None, k=None, RightMotor=None, LeftMotor=None, Reset=True, SpeedScale=1, Timeout=999000):
            global OverallScale, PIDStopper
            if StopID:
                PIDStopper[StopID] = False
            SpeedScale *= OverallScale
            TimeoutTimer = Timer()
            print("Drive", Direction, TargetDistance, "inches")
            if not RightMotor:
                RightMotor = self.DefaultRight
            if not LeftMotor:
                LeftMotor = self.DefaultLeft
            if Reset:
                LeftMotor.reset_position()
                RightMotor.reset_position()
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
                RightPos = RightMotor.position(DEGREES) * (-1 if Direction == REVERSE else 1)
                LeftPos = LeftMotor.position(DEGREES) * (-1 if Direction == REVERSE else 1)
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
        def Turn(self, Direction, TargetPos, StopID=None, k=None, RightMotor=None, LeftMotor=None, Reset=True, SpeedScale=1, Timeout=999000):
            global OverallScale, PIDStopper
            if StopID:
                PIDStopper[StopID] = False
            SpeedScale *= OverallScale
            TimeoutTimer = Timer()
            print("Turn", Direction, TargetPos, "Degrees")
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
            return
        def SpinMotor(self, Motor_, Direction, Distance_, StopID=None, k=None, GearRatio=1, Unit=DEGREES, Reset=True, SpeedScale=1, Timeout=999000):
            global OverallScale, PIDStopper
            if StopID:
                PIDStopper[StopID] = False
            SpeedScale *= OverallScale
            print("Spin", Motor_, Direction, Distance_, "Degrees")
            TimeoutTimer = Timer()
            if Reset:
                Motor_.reset_position()
            if not k:
                k = self.k
            kP = k[0]
            kI = k[1]
            kD = k[2]
            Integral = 0
            LastError = 0
            while TimeoutTimer.time(MSEC) < Timeout:
                Pos = Motor_.position(Unit) * GearRatio
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
        def __init__(self, RobotObject, x=None, y=None, Units=INCHES):
            self.Robot = RobotObject
            self.Units = Units
            self.InertialObject = Inertial()
            if x:
                self.x = x
            if y:
                self.y = y
            self.Reset()

        def TrackLocation(self):
            while True:
                
            
        def Reset(self):
            self.x = 8.5
            self.y = 9.375
            self.InertialObject.reset_heading()
        def Angle(self):
            return self.InertialObject.heading(DEGREES)

        def ToPoint(self, Point, Direction=FORWARD, SpeedScale=1, TurnScale=1, DriveScale=1, DriveTimeout=999000):
            x_loc, y_loc = Point
            while True:
                angle_to_turn_to = math.radians(math.atan2(y_loc - self.y, x_loc - self.x))
                if Direction == REVERSE:
                    angle_to_turn_to += 180
                # - Turn -
                degrees_to_turn = (angle_to_turn_to - self.Angle()) % 360
                if degrees_to_turn > 180:
                    degrees_to_turn -= 360
                if degrees_to_turn < -180:
                    degrees_to_turn += 360
                if degrees_to_turn < 0:
                    turn_angle = LEFT
                    degrees_to_turn *= -1
                else:
                    turn_angle = RIGHT
                
                if abs(degrees_to_turn) > 2:
                    self.Robot.PID.Turn(turn_angle, degrees_to_turn, SpeedScale=(SpeedScale * TurnScale))

                # - Drive -
                Distance = math.sqrt((x_loc - self.x) ** 2 + (y_loc, self.y) ** 2)

                self.Robot.PID.Drive(Direction, Distance)






Robot = Init()
Robot.PID = Robot.InitPID(0.34, 0.002, 0.55, 2.5, 2.5, Drive=(Robot.DriveMotors.Right, Robot.DriveMotors.Left))
Robot.PTP = Robot.InitPTP(Robot)

def Clamp(Num, Max=100, Min=-100):
    return max(min(Num, Max), Min)

def Min(Num, Lim=7.5):
    if Num < 0:
        return min(Num, -Lim)
    else:
        return max(Num, Lim)





def Autonomous():
    global OverallScale, PIDStopper
    CreateThread = Event()
    cols = [Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN, Color.BLUE, Color.PURPLE]
    while not Robot.StartButton.pressing():
        Robot.StartButton.set_color(cols[urandom.randint(1, len(cols)) - 1])
        wait(500, MSEC)
    Robot.StartButton.set_fade(FadeType.OFF)
    Robot.StartButton.on(Color.RED)
    while Robot.StartButton.pressing():
        pass
    Robot.StartButton.set_brightness(50)
    Robot.StartButton.set_blink(Color.GREEN, 0.75, 1.25)
    Robot.Start()

    # ---------------------- Starting Autonomous Code ----------------------

    print("[DEBUG] Autonomous Starting...")
    print("\n----- Autonomous -----\n")


    # -- Get First Pins --

    Robot.PID.Drive(FORWARD, 15.5, SpeedScale=0.7)
    Robot.Pin.Grab()
    Robot.PID.Turn(LEFT, 23.5, SpeedScale=1.2)
    Robot.Pin.Release()
    Robot.PID.Drive(FORWARD, 7, SpeedScale=0.8)
    Robot.Pin.Grab()
    Robot.PID.Turn(LEFT, 40, SpeedScale=1.3)
    Robot.PinArm.set_stopping(HOLD)
    Robot.BeamArm.set_stopping(HOLD)




    # -- Get Second Pins --

    Robot.PID.SpinMotor(Robot.PinArm, FORWARD, 50, GearRatio=(1 / 3), SpeedScale=8)
    Robot.PID.Drive(FORWARD, 12, SpeedScale=0.9)
    Robot.PID.Turn(RIGHT, 20, SpeedScale=1.5)
    Robot.PID.Drive(FORWARD, 2.2, SpeedScale=0.8)
    Robot.PID.Turn(RIGHT, 9.5, SpeedScale=1.5)
    Robot.PID.Drive(FORWARD, 15, SpeedScale=0.8)


    # -- Stack --

    Robot.PinArm.set_velocity(76, PERCENT)
    Robot.PinArm.set_max_torque(8, PERCENT)
    Robot.PinArm.spin(REVERSE)
    Robot.PID.Drive(FORWARD, 1.5, SpeedScale=0.6)
    Robot.Pin.Release()
    Robot.PinArm.set_max_torque(100, PERCENT)
    Robot.PID.SpinMotor(Robot.PinArm, REVERSE, 20, GearRatio=(1 / 3), SpeedScale=7, Timeout=500)
    Robot.PID.Drive(FORWARD, 2, SpeedScale=0.9)
    Robot.Pin.Grab()


    # -- Get Beam --

    Robot.PID.Turn(RIGHT, 78.5, SpeedScale=1.2)
    Robot.PID.SpinMotor(Robot.PinArm, FORWARD, 40, SpeedScale=6)
    Robot.PID.Drive(FORWARD, 14.85, SpeedScale=0.8)
    Robot.PID.Turn(RIGHT, 53.5, SpeedScale=1.35)
    wait(200, MSEC)
    Robot.DriveMotors.Main.set_velocity(100, PERCENT)
    Robot.DriveMotors.Main.spin(REVERSE)
    wait(1500, MSEC)
    Robot.DriveMotors.Main.stop()
    Robot.PID.Drive(REVERSE, 2, SpeedScale=0.7, Timeout=250)
    Robot.Beam.Grab()


    # -- Flip Pins --

    Robot.PID.Drive(FORWARD, 4, SpeedScale=0.8)
    Robot.PID.SpinMotor(Robot.BeamArm, FORWARD, 14, GearRatio=(1 / 5), SpeedScale=3, Timeout=750)
    Robot.PID.SpinMotor(Robot.PinArm, FORWARD, 200, GearRatio=(1 / 3), SpeedScale=5, Timeout=2000)
    wait(60, MSEC)
    Robot.Pin.Release()
    wait(140, MSEC)
    Robot.PinArm.set_velocity(100, PERCENT)
    Robot.PinArm.spin(REVERSE)
    wait(2000, MSEC)
    Robot.PinArm.stop()


    # -- Put Pin on Standoff --

    Robot.PID.SpinMotor(Robot.BeamArm, FORWARD, 120, GearRatio=(1 / 5), SpeedScale=4, Timeout=2000)
    Robot.Pin.Grab()
    Robot.PID.Drive(FORWARD, 4, SpeedScale=1.2)
    Thread(Robot.StopSpinning)
    Robot.PID.Turn(RIGHT, 19, SpeedScale=4.8, StopID="SpinningFunction")
    Robot.Pin.Release()
    Robot.PID.Drive(FORWARD, 21, SpeedScale=0.6, Timeout=1400)
    wait(75, MSEC)
    if Robot.Dist.object_distance(MM) > 100:
        # Missed Standoff
        Robot.PID.Drive(REVERSE, 10, SpeedScale=1.5)
        Thread(Robot.StopSpinning)
        Robot.PID.Turn(RIGHT, 19, SpeedScale=4.8, StopID="SpinningFunction")
        Robot.Pin.Release()
        Robot.PID.Drive(FORWARD, 21, SpeedScale=0.6, Timeout=1400)
        wait(75, MSEC)
    Robot.PID.Drive(REVERSE, 8, SpeedScale=1.4)


    # -- Put Beam on Standoff Pin --

    Robot.PID.Turn(RIGHT, 159, SpeedScale=2.1)
    Robot.PID.Drive(REVERSE, 30, SpeedScale=0.95, Timeout=1200)
    Robot.PID.Drive(REVERSE, 30, SpeedScale=1.3, Timeout=1350)
    ReverseDrive = CreateThread(Robot.PID.Drive, (REVERSE, 30, 1))
    CreateThread.broadcast()
    Robot.PID.SpinMotor(Robot.BeamArm, REVERSE, 28, GearRatio=(1 / 5), SpeedScale=5, Timeout=545)
    Robot.Beam.Release()
    Robot.PID.SpinMotor(Robot.BeamArm, REVERSE, 4, GearRatio=(1 / 5), SpeedScale=5, Timeout=750)
    wait(300, MSEC)
    Robot.PID.SpinMotor(Robot.BeamArm, FORWARD, 20, GearRatio=(1 / 5), SpeedScale=3, Timeout=750)
    PIDStopper[1] = True
    Robot.PID.Drive(FORWARD, 15)


    return



def main():
    VidTimer = Timer()
    AutoThread = Thread(Autonomous)
    #brain.buttonCheck.pressed(Autonomous2)


main()
