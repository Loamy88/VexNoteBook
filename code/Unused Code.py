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







    def ToPoint(self, Point, Direction=FORWARD, StopSmooth=False, SpeedScale=1, TurnScale=1, DriveScale=1):
        global PIDDriveScale, PIDValues
        PIDDriveScale = SpeedScale * DriveScale
        TargetX, TargetY = Point
        if self.Debug:
            print("\033[0m - Driving from (", round(self.x, 2), ", ", round(self.y, 2), ") to (", TargetX, ", ", TargetY, ") -", sep="") # Driving from (x, y) to (x, y)
    
        IsDriving = False
        self.RunningOdom = True
        self.DriveThread = None
        Distance = math.sqrt((TargetX - self.x) ** 2 + (TargetY - self.y) ** 2)
        if Distance < self.Margin * 0.95:
            self.RunningOdom = False

        while self.RunningOdom:

            angle_to_turn_to = round(math.atan2(TargetY - self.y, TargetX - self.x), 4)
            current_angle = round(brain_inertial.heading(TURNS) * math.pi * 2, 4)
            Distance = math.sqrt((TargetX - self.x) ** 2 + (TargetY - self.y) ** 2)
            if Direction == REVERSE:
                angle_to_turn_to += math.pi
            radians_to_turn = round((angle_to_turn_to - current_angle) % (2 * math.pi), 4)

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

            radians_to_turn = round(radians_to_turn, 4)
            
            if abs(radians_to_turn) > max(math.tan(self.Margin / Distance) * 1.3, 0.18):
                # If the angle is off then stop driving forward and turn
                if self.Debug:
                    print("\033[32mCurrent Angle:", current_angle, "Angle to Point:", angle_to_turn_to, "- Adjusting Angle")
                if self.DriveThread != None:
                    self.DriveThread.stop()
                IsDriving = False
                self.StopDrivingSmooth()
                wait(90, MSEC)
                Robot.PIDTurn(turn_direction, radians_to_turn, SpeedScale=(SpeedScale * TurnScale * 1.7))
                wait(90, MSEC)
            elif abs(radians_to_turn) > 0.1 and IsDriving and "OdomDrive" in PIDValues.keys():
                # Slightly adjust angle while driving
                # 305.6 is the rotations to spin one motor for turning one radian converted into percent (using 35 MSEC PID cycles)  <- future referance
                print("nudging", radians_to_turn)
                if radians_to_turn > 0:
                    # Turn Left
                    Robot.DriveLeft.set_velocity(Robot.DriveLeft.velocity(PERCENT) + min(radians_to_turn * 180.6, 5.6), PERCENT)
                if radians_to_turn < 0:
                    # Turn Left
                    Robot.DriveRight.set_velocity(Robot.DriveRight.velocity(PERCENT) + min(radians_to_turn * 180.6, 5.6), PERCENT)
                if PIDValues["OdomDrive"]["direction"] == FORWARD:
                    PIDValues["OdomDrive"]["distance"] = Distance
                else:
                    PIDValues["OdomDrive"]["distance"] = -Distance

            if not IsDriving:
                # Drive if the robot isn't already doing so

                self.DriveThread = Thread(Robot.PIDDrive, (Direction, Distance, "OdomDrive"))
                while not "OdomDrive" in PIDValues.keys():
                    wait(2, MSEC)
                PIDValues["OdomDrive"]["speed"] *= SpeedScale * TurnScale
                IsDriving = True

                # Check for stopping the Odom
                if Distance < (self.Margin * 0.95):
                    self.RunningOdom = False

            elif "OdomDrive" in PIDValues.keys():
                # Update if drive thread has died out on it's own
                IsDriving = PIDValues["OdomDrive"]["running"]

        if self.DriveThread != None:
            self.DriveThread.stop()
        self.RunningOdom = False
        if StopSmooth:
            self.StopDrivingSmooth()
    
    def StopDrivingSmooth(self, Rate=0.994):
        CurrentVelocity = 100
        while CurrentVelocity > 10:
            CurrentVelocity = (Robot.DriveLeft.velocity(PERCENT) + Robot.DriveRight.velocity(PERCENT)) / 2
            Robot.DriveRight.set_velocity(CurrentVelocity * Rate, PERCENT)
            Robot.DriveLeft.set_velocity(CurrentVelocity * Rate, PERCENT)