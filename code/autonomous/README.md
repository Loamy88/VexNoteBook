# Code Details and Versions

- [Download Main Code](https://loamy88.github.io/VexNoteBook/code/autonomous/180%20Flip.py) - ([View On Github](./180%20Flip.py))
  
  
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
