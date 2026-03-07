# Code Details and Versions  

## Version 1.0.0:  

- Initial setup on github  


### Main Features: 

1. Init class:  
    - Sets up the python objects for motors, pneumatics, ect.  
    - Includes functions to easily control them  
    - Currently unused PID controller - may be inplemented later  

2. Driving thread:  
    - Uses drive style - arcade or tank  
    - Calculates power per side based on controller values and drive style  
    - 0 power if the controller stick is in the deadzone  
    - Applies power to each side  

3. Arm control thread:  
    - Lifts and lowers the beam arm with R-Up and R-Down  
    - Lifts and lowers the pin arm with L-Up and L-Down  
    - Controls stopping arms  
    - Applies whether something is being "auto-lifted/flip" to disable other controls  

4. Claw control thread:  
    - Opens and closes the beam claw with F-Up and F-Down  
    - Opens and closes the pin claw with E-Up and E-down

5. Callback functions:
    - Shift button is currently L3  
    - R3 switches the drive style between tank and arcade  
    - R-Down with shift pressed toggles the aligner for placing beams on 2 stacks  
    - L-Down with shift pressed toggles BAS  
    - R-Up with shift pressed will automaticly fully lift or lower the beam arm - uses a P controller for lifting to keep the beam stable  
    - L-Up with shift pressed will automaticly fully flip the pin arm  

6. Stop thread:
    - Checks for both L3 and R3 being pressed for ~2 seconds then stops the program  
    - Does a battery check after ~1 second of being pressed  