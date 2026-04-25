# Code Details and Versions

- Download Old Code as [.py](https://loamy88.github.io/VexNoteBook/code/autonomous/110%20Autonomous.py) or [.iqpython](https://loamy88.github.io/VexNoteBook/code/autonomous/110%20-%20Autonomous%20-%20Working.iqpython) - ([View On Github](./110%20Autonomous.py))
- Download New Code as [.py](https://loamy88.github.io/VexNoteBook/code/autonomous/New%20Autonomous.py) or [.iqpython](https://loamy88.github.io/VexNoteBook/code/autonomous/PTP%20Autonomous.iqpython) - ([View On Github](./New%20Autonomous.py))
- [Return to Code Page](../)
  
  
---

## Version 2.1.0 (April 25, 2026):

- Removed `Odom.ToPoint`

### Reasons:

1. Inaccuracies:
    - No matter how much tuning I did, the PID values and nudging values were never quite right
    - This caused the robot to always to driving to the point and start slightly adjusting itself for a while
    - The robot could never drive perfectly to the point and stop, especially not hitting it at the right angle

2. Nudging and Tracking not Working Well Together:
    - As I found out earlier, the robot can't turn with the perfect accuracy needed to drive perfectly to the point, so nudging was needed
    - When the nudging moved the robot, it would apply slight power to one side to turn the robot while driving
    - The tracking is built to track consistent movement between sides
    - The nudging should have been small enough, but over time it seemed to add significant error to the tracking

3. Time Contraints:
    - Today (April 25th) is the day before we leave for worlds
    - We need to get at least some points for autonomous
    - There isn't enough time for perfectly tuning `Odom.ToPoint`

3. Alternatives:
    - Previously, autonomous just used PID with pre-defined movements, but over time it got too far off
    - Now, we can try to simply do the same, but at certain places fix the position by getting `Odom.x` and `Odom.y` and adjust the position to get it perfect

## Version 2.0.1 (April 14, 2026):

- Added correction for small direction errors while odom is driving
    - The PID turn isn't accurate enough to perfectly face the point from far away
    - Lots of time is lost if the odom has to stop and correct while driving
    - If there is small error, the odom adds slightly more power to one side to turn it before the error increases

## Version 2.0.0 (April 12, 2026):  

- New autonomous system: Odometry 

### Main Features:  

1. New Paths:  
    - This new system might allow for a longer route  
    - When developing the the route, we might extend it beyond simply 110 points  

2. Odometry Autonomous System:  
    - Tracks the location and heading using inertial 
    - When tracking location, sensors hitting a black line will adjust the position to be more accurate  
    - Calculates the angle and distance to travel to the next point  
        - Checks to see if the angle is off enough with `abs(degrees_to_turn) > max(math.tan(self.Margin / math.sqrt((x_loc - self.x) ** 2 + (y_loc - self.y) ** 2)), 1.8)` to decide to turn or not
        - A better *but messy* representation of this in desmos can be found [here](https://www.desmos.com/calculator/uxkigfllwh)
    - Uses PID to accurately drive set distances and to turn
    - In the future, we might add optical sensors to look for lines on the game field and use that to correct the position

3. Driving Route:
    - We currently don't have the path set up
    - The new system uses function calls like this:
    ```python
    TrackingThread = Thread(Odom.TrackLocation)
    # Driving to Pin
    Odom.ToPoint((1, 8.5), SpeedScale=1.5)
    ```

4. Optimization:
    - Recently, the autonomous code has been getting large (~20KB with memory intensive code) and we have been getting memory allocation errors on program startup
    - Reducing the excessive use of some functions and classes has helped to lower memory usage

5. PID:
    - The PID has lately had some accuracy issues
    - The turn and drive PID values are retuned to ensure no problems
    - Spinning not driving motors has turned out to not require the accuracy acheived by PID, so that functions has been replaced by a simple P controller
    - Graph of the driving error: 
    - ![PID Drive Error](./graphs/PID_drive_error.png)
    - Graph of the turning error: 
    - ![PID Turn Error](./graphs/PID_turn_error.png)

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
