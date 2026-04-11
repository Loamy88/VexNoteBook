# Code Details and Versions

- Download Main Code as [.py](https://loamy88.github.io/VexNoteBook/code/180%20Flip.py) or [.iqpython](https://loamy88.github.io/VexNoteBook/code/180%20Flip.iqpython) - ([View On Github](./180%20Flip.py))  
- Download Code for New Robot as [.py](https://loamy88.github.io/VexNoteBook/code/New%20180%20Flip%20-%20Not%20Used.py) or [.iqpython](https://loamy88.github.io/VexNoteBook/code/180%20Flip%20-%20V.2.iqpython) - ([View On Github](./New%20180%20Flip%20-%20Not%20Used.py))
- [Autonomous Code Page](./autonomous)

---

## Version 1.1.3 (April 11, 2026):

- Make L3 activate any aligners that have their corresponding arms raised

## New Robot Version 0.2.1 (April 7, 2026):

- Make Shift L-Up and R-Up toggle their respective aligners
- Make PAS activate when the pin arm is lowered down

## Version 1.1.2 (April 7, 2026):

- Make Shift L-Up and R-Up toggle their respective aligners

## New Robot Version 0.2.0 (April 2, 2026):

- Update ports for testing from main code for the new robot

## Version 1.1.1 (March 30, 2026):

- Added a loop to the main thread to track wheel movement, persisting through program runs. It is saved to the sd if it is inserted with `brain.sdcard.isinserted()`

## Version 1.1.0 (March 17, 2026):  

- Make F-Down and E-Down (back paddles) toggle claws
- Make F-Up and E-Up (upper paddles) toggle aligners
- Works better with the new controller attachment

### Main Features:

1. Ready Checks:
    - For each button, the program saves if it is ready to be pressed
    - The action for the button only happens if it is ready, and it will make no longer ready
    - If the button isn't pressed, it will become ready again
    - This insures that each time it is pressed, the action only happens once

2. Checking for Open Claws:
    - The program uses `Robot.Claws.status()` to check if the claws are open
    - It uses a bitwise `&` to see if the returned integer has the bits to signify an extended cylinder:
    ```python
    if Robot.Claws.status() & 768 == 768:
        # Cylinder 1 is open
        Robot.Claws.retract(Robot.Pin)
    else:
        # Cylinder 1 is closed
        Robot.Claws.extend(Robot.Pin)
    ```
    - It will then open or close the claw
    
## Version 1.0.1 (March 12, 2026):

- Shift L-Down now toggles our active pin aligner

## Version 1.0.0 (March 6, 2026):  
  
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
    - Applies whether something is being "auto-lifted/flipped" to disable other controls  
  
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