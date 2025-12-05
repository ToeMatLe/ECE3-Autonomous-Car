# ECE3-Autonomous-Car
Final Project for the UCLA ECE3 course. The robot was tuned for speed, stability, and precise checkpoint handling, achieving the fastest completion time of 5.6 seconds in a class of 196 students.
This project implements a fully autonomous line-following robot using IR sensors, motor encoders, a PID controller, and a multi-stage state machine. The robot follows a track, detects multiple “checkpoint” black lines, performs a 225° turn, accelerates through a timed segment, and finally stops itself using encoder feedback. 
The code is written for the ECE3 robot platform and uses the ECE3.h library for IR sensor reading and encoder access.

Board:
- RED LaunchPad w/ msp432p401r EMT (48MHz)

Calibrations:
- Found Fusion Output:
  - Each IR channel is mapped from calibrated min/max to a normalized 0–1000 scale:
    - vi = 1000 * (si​−mini)/(maxi-mini)
  - Weighted sum of sensors produces the lateral error signal:
    -  e = 1/8 * sum(wi*vi)
  - PID Steering
    - u=Kp​e+Kd​(e−eprev​)
    - vL​=baseSpeed−u,vR​=baseSpeed+u
