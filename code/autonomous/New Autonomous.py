#region
from vex import *
import urandom
import math

brain=Brain()

brain_inertial = Inertial()



def initializeRandomSeed():
    wait(100, MSEC)
    xaxis = brain_inertial.acceleration(XAXIS) * 1000
    yaxis = brain_inertial.acceleration(YAXIS) * 1000
    zaxis = brain_inertial.acceleration(ZAXIS) * 1000
    systemTime = brain.timer.system() * 100
    urandom.seed(int(xaxis + yaxis + zaxis + systemTime)) 
    
initializeRandomSeed()

#endregion

# ------------------------------------------
# 
# 	Project:      United Robotics 180 Flip Code
#   Team:         26277E
# 	Author:       Euan O'Brien
# 	Description:  VEX IQ python program for advanced autonomous
# 
# ------------------------------------------




OverallScale = 1.1
PIDDriveScale = 1.0
PIDTurnScale = 1.0



version = "2.1.1"

print("180 Flip Autonomous Code Version:", version)




# Initialize Motors

class Init:
    def __init__(self):
        if brain.battery.capacity() <= 75:
            self.LowBat()

        self.StartButton = Touchled(Ports.PORT6)
        self.StartButton.set_fade(FadeType.SLOW)
        self.StartButton.on(Color.YELLOW)

        self.BeamArm = MotorGroup(Motor(Ports.PORT7), Motor(Ports.PORT1, True))
        self.BeamArm.set_stopping(HOLD)
        self.BeamArm.stop()

        self.Claws = Pneumatic(Ports.PORT11)
        self.Claws.pump_off()
        self.Claws.retract(CYLINDER1)
        self.Claws.retract(CYLINDER2)

        self.PinArm = MotorGroup(Motor(Ports.PORT10), Motor(Ports.PORT4, True))
        self.PinArm.set_stopping(HOLD)
        self.PinArm.set_position(0, DEGREES)
        self.PinArm.stop()

        for motor in (self.BeamArm, self.PinArm):
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
    def LowBat(self):
        batt = brain.battery.capacity()
        note = round(batt / 20)
        brain.play_note(3, note, 500)
        brain.screen.print("Low Battery: " + str(batt))
    def Claw(self, Cylinder, Grab):
        if Grab:
            self.Claws.extend(Cylinder)
        else:
            self.Claws.retract(Cylinder)


    def InitPID(self, K, KTurn, Pin, Beam, Drive=(None, None)):
        self.K = K
        self.KTurn = KTurn
        self.KpPin = Pin
        self.KpBeam = Beam
        if Drive:
            self.DefaultRight, self.DefaultLeft = Drive
    def PIDDrive(self, Direction, Target, Reset=True, StationaryWaitTime=120, SpeedScale=1, Timeout=999000):
        global OverallScale, PIDDriveScale
        
        Speed = SpeedScale * OverallScale * PIDDriveScale
        TimeoutTimer = Timer()
        StationaryTime = TimeoutTimer.time(MSEC)
        IsStationary = True
        #print("\033[32mDrive", Direction, Target, "radians?")
        RightMotor = self.DefaultRight
        LeftMotor = self.DefaultLeft
        if Reset:
            LeftStart = LeftMotor.position(TURNS) * math.pi * 2
            RightStart = RightMotor.position(TURNS) * math.pi * 2
        K = self.K
        Kp = K[0]
        Ki = K[1]
        Kd = K[2]
        RightIntegral = 0
        LeftIntegral = 0
        RightLastError = 0
        LeftLastError = 0
        if Direction == REVERSE:
            Target *= -1
        
        CurrentTime = TimeoutTimer.time(MSEC)

        self.PIDRunning = True
        

        while CurrentTime < Timeout:
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

            Difference = RightError - LeftError

            # Calculate power and move the robot
            RightPower = (RightError * Kp) + (RightIntegral * Ki) + (RightDerivative * Kd) + (Difference * Kp / 4)
            LeftPower = (LeftError * Kp) + (LeftIntegral * Ki) + (LeftDerivative * Kd) - (Difference * Kp / 4)
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
                    self.IsRunning = False
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
        self.IsRunning = False
        return
    def PIDTurn(self, Direction, TargetPos, StationaryWaitTime=120, Reset=True, SpeedScale=1, Timeout=999000):
        global OverallScale, PIDTurnScale
        SpeedScale *= OverallScale * PIDTurnScale
        TimeoutTimer = Timer()
        StationaryTime = TimeoutTimer.time(MSEC)
        IsStationary = False
        RightMotor = self.DefaultRight
        LeftMotor = self.DefaultLeft
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
        else:
            Start = 0.0
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
            RightMotor.set_velocity(Min(Clamp(Power * SpeedScale * RightDir)), PERCENT)
            LeftMotor.set_velocity(Min(Clamp(Power * SpeedScale * LeftDir)), PERCENT)
            RightMotor.spin(FORWARD)
            LeftMotor.spin(FORWARD)

            if abs(Error) < 0.08:
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
        if Arm == 1:
            Motor_ = self.PinArm
            GearRatio = (1 / 3)
            if not Kp:
                Kp = self.KpPin
        elif Arm == 2:
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
            
            wait(9, MSEC)
            
        Motor_.stop()
        del TimeoutTimer
        return


class InitOdometry:
    def TrackLocation(self):
        self.Tracking = True

        LastMotorPos = (Robot.DriveLeft.position(TURNS) + Robot.DriveRight.position(TURNS)) * math.pi

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

            
    def AngleToPoint(self, Point, Direction=FORWARD):
        TargetX, TargetY = Point    
        angle_to_turn_to = math.atan2(TargetY - self.y, TargetX - self.x)
        current_angle = brain_inertial.heading(TURNS) * math.pi * 2
        Distance = math.sqrt((TargetX - self.x) ** 2 + (TargetY - self.y) ** 2)
        if Direction == REVERSE:
            angle_to_turn_to += math.pi
        radians_to_turn = (angle_to_turn_to - current_angle) % (2 * math.pi)

        # Get optimal turn direction
        if radians_to_turn > math.pi:
            radians_to_turn -= 2 * math.pi
        if radians_to_turn < -math.pi:
            radians_to_turn += 2 * math.pi
        if radians_to_turn < 0:
            turn_direction = LEFT
            radians_to_turn *= -1
        else:
            turn_direction = RIGHT

        return turn_direction, radians_to_turn

        
        
    def Reset(self):
        self.x, self.y = 0.0, 0.0
        brain_inertial.set_heading(0, DEGREES)
        brain_inertial.set_rotation(0, DEGREES)





def Clamp(Num, Max=100, Min=-100):
    return max(min(Num, Max), Min)

def Min(Num, Lim=7.5):
    if Num < 0:
        return min(Num, -Lim)
    else:
        return max(Num, Lim)



Robot = Init()
Robot.InitPID((12.95, 0.093, 20.0), (26.6, 0.0155, 114.5), 1.05, 0.9, Drive=(Robot.DriveRight, Robot.DriveLeft))
Odom = InitOdometry()



def LowerBeam():
    wait(500, MSEC)
    Robot.BeamArm.spin(REVERSE)
    wait(500, MSEC)
    Robot.BeamArm.set_stopping(COAST)
    Robot.BeamArm.spin(FORWARD)
    wait(40, MSEC)
    Robot.BeamArm.stop()
    



def Autonomous():
    global OverallScale, PIDTurnScale
    PIN = CYLINDER2
    BEAM = CYLINDER1
    PINARM = 1
    BEAMARM = 2
    Robot.StartButton.set_color(Color.BLUE)
    Robot.Claws.pump_on()
    while not Robot.StartButton.pressing():
        wait(40, MSEC)
    Robot.StartButton.set_fade(FadeType.OFF)
    Robot.StartButton.on(Color.RED)
    brain_inertial.set_heading(0, DEGREES)
    Odom.Reset()
    while Robot.StartButton.pressing():
        pass
    Robot.StartButton.set_brightness(50)
    Robot.StartButton.set_blink(Color.GREEN, 0.75, 1.25)
    AutoTimer = Timer()
    AutoTimer.event(brain.program_stop, 59800)
    TrackingThread = Thread(Odom.TrackLocation)
    

    # ---------------------- Starting Autonomous Code ----------------------

    
    


    # -- Get First Pins --

    Thread(LowerBeam)
    Robot.PIDDrive(FORWARD, 3.18)
    Robot.PIDTurn(LEFT, 0.2)
    Robot.PIDDrive(FORWARD, 5.05)
    #Robot.Claw(PIN, True)
    Robot.PIDTurn(RIGHT, 0.68)
    Robot.PIDDrive(FORWARD, 3.4)
    TurnDirection, TurnAngle = Odom.AngleToPoint((14.25, 3.37))
    Robot.PIDTurn(TurnDirection, TurnAngle)
    #Robot.Claw(PIN, False)
    Robot.PIDDrive(FORWARD, 5.5, SpeedScale=0.86)
    Robot.Claw(PIN, True)
    Robot.PIDDrive(REVERSE, 0.5)
    PIDTurnScale = 1.1


    # -- Get Second Pins --

    Robot.SpinArm(PINARM, 55)
    Robot.PIDDrive(FORWARD, 1.85)
    Robot.PIDTurn(RIGHT, 0.7)
    Robot.PIDDrive(FORWARD, 4.25)
    Robot.PIDTurn(RIGHT, 0.39)
    Robot.PIDDrive(FORWARD, 3.1)


    # -- Stack --

    Robot.SpinArm(PINARM, -45, Timeout=500)
    Robot.Claw(PIN, False)
    Robot.SpinArm(PINARM, -35, Timeout=300)
    Robot.PIDDrive(FORWARD, 0.2)
    Robot.Claw(PIN, True)
    Robot.SpinArm(PINARM, 30)
    PIDTurnScale = 1.2


    # -- Get Beam --

    Robot.PIDDrive(REVERSE, 0.9)
    Robot.PIDTurn(RIGHT, (math.pi / 2), Reset=False)
    DistanceToPoint = 11.1 - Odom.y
    Robot.PIDDrive((FORWARD if DistanceToPoint > 0.0 else REVERSE), abs(DistanceToPoint))
    Robot.PIDTurn(RIGHT, math.pi, Reset=False)
    Robot.PIDDrive(REVERSE, 2.98, Timeout=1300) # should drive 2.42
    Robot.Claw(BEAM, True)
    PIDTurnScale = 1.3


    # -- Flip Pins --

    Robot.SpinArm(PINARM, 180, Timeout=1400)
    Robot.Claw(PIN, False)
    Robot.SpinArm(PINARM, -200, Timeout=1400)


    # -- Put Pin on Standoff --

    Robot.BeamArm.set_stopping(HOLD)
    Robot.PIDDrive(FORWARD, 1.08)
    Robot.SpinArm(BEAMARM, 150, Timeout=1600)
    TurnDirection, TurnAngle = Odom.AngleToPoint((10.5, 11.5))
    TurnAngle += -0.14 if TurnDirection == RIGHT else 0.14
    if TurnAngle > 0.12:
        Robot.PIDTurn(TurnDirection, TurnAngle, SpeedScale=1.2)
    Robot.PIDDrive(FORWARD, 5.0, Timeout=1700) # should drive 4.28
    Robot.PIDDrive(REVERSE, 2.85, SpeedScale=1.1)
    PIDTurnScale = 1.4


    # -- Put Beam on Standoff Pin --

    Robot.PIDTurn(RIGHT, 3.5, SpeedScale=1.1)
    Robot.PIDDrive(REVERSE, 3.35, Timeout=1900) # should drive 2.45
    Robot.BeamArm.set_velocity(40, PERCENT)
    Robot.BeamArm.spin(REVERSE)
    wait(300, MSEC)
    Robot.Claw(BEAM, False)
    Robot.BeamArm.stop()
    Robot.SpinArm(BEAMARM, 20, Timeout=100)
    Odom.x = 10.5 + (math.cos(brain_inertial.heading(TURNS) * 2 * math.pi) * 2.5)
    Odom.y = 11.5 + (math.sin(brain_inertial.heading(TURNS) * 2 * math.pi) * 2.5)
    PIDTurnScale = 1.1
    Robot.PIDDrive(FORWARD, 3.2)


    # -- Get Extra Stack --

    TurnDirection, TurnAngle = Odom.AngleToPoint((14.4, 17.0))
    Robot.PIDTurn(TurnDirection, TurnAngle)
    PointDistance = math.sqrt((Odom.x - 14.4) ** 2 + (Odom.y - 17.0) ** 2)
    Robot.PIDDrive(FORWARD, PointDistance)
    Robot.PIDTurn((RIGHT if brain_inertial.heading(TURNS) * 2 * math.pi < 8.1 else LEFT), 8.1, Reset=False)
    Robot.PIDDrive(FORWARD, 2.8)
    Robot.Claw(PIN, True)
    Robot.SpinArm(PINARM, 48)
    Robot.PIDTurn(LEFT, 1.23)
    Robot.PIDDrive(FORWARD, 2.0)
    Robot.PinArm.set_stopping(COAST)
    Robot.SpinArm(PINARM, -40, Timeout=600)
    Robot.PIDTurn(LEFT, 0.5)
    Robot.PIDDrive(FORWARD, 5.3, Timeout=1200) # should drive 4.5


    brain.play_note(3,0,400)
    brain.screen.print(str(AutoTimer.time(MSEC)))
    return# brain.program_stop()



def main():
    Robot.BeamArm.set_stopping(HOLD)

    # calibrate
    brain_inertial.calibrate()
    while brain_inertial.is_calibrating():
        wait(10, MSEC)
    wait(100, MSEC)
    brain.play_note(3, 4, 50)
    brain.play_note(3, 4, 200)

    # wait for first press to start
    while not Robot.StartButton.pressing():
        wait(10, MSEC)

    # fix slack
    wait(100, MSEC)
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

    while Robot.StartButton.pressing():
        wait(10, MSEC)

    # check for calibration falure
    if brain_inertial.heading(TURNS) == 0.0:
        brain.program_stop()

    # run auton


main()
Autonomous()
