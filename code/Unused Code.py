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