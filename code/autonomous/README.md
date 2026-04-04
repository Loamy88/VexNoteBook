# Code Details and Versions

- [Download Main Code](https://loamy88.github.io/VexNoteBook/code/autonomous/110%20Autonomous.py) - ([View On Github](./110%20Autonomous.py))
  
  
## Version 1.0.0 (March 6, 2026):  
  
- Initial setup on github  
  
  
### Main Features:  
1. PID Controller:
    - Creates functions for driving, turning, and spinning arm motors
    - Uses the error of the motor multiplied by the p factor for the base power  
    - Adds the cumulative error multiplied by the i factor  
    - Adds the change in error multiplied by the d factor  
    - Applies the power  
    - This makes the motors spin slower as they approach the target, making driving more accurate  
  
2. Using the PID to Follow the Driving Path:  
    - Calls the PID functions to follow the predefined route  
    - Uses other function calls to open and close claws  


## Version 2.0.0 (April XX, 2026) - WORK IN PROGRESS (Listed Goals):  

- New autonomous system: Point-To-Point (PTP)  

### Main Features:  

1. New Paths:  
    - This new system might allow for a longer route  
    - When developing the the route, we might extend it beyond simply 110 points  

2. Point-To-Point (PTP) Autonomous System:  
    - Tracks the location and heading  
    - When trcaking location, sensors hitting a black line will adjust the position to be more accurate  
    - Calculates the angle and distance to travel to the next point  
    - Uses PID to accurately drive set distances and to turn

3. Driving Route:
    - We currently don't have the path set up

4. Optimization:
    - Recently, the autonomous code has been getting large (~20KB) and we have been getting memory allocation errors on program startup
    - Reducing the excessive use of some functions and classes has helped to lower memory usage
