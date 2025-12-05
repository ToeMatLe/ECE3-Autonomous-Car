# ECE3-Autonomous-Car
Final Project for the UCLA ECE3 course. The robot was tuned for speed, stability, and precise checkpoint handling, achieving the fastest completion time of 5.6 seconds in a class of 196 students.

This project implements a fully autonomous line-following robot using IR sensors, motor encoders, a PID controller, and a multi-stage state machine. The robot follows a track, detects multiple “checkpoint” black lines, performs a 225° turn, accelerates through a timed segment, and finally stops itself using encoder feedback. 

The code is written for the ECE3 robot platform and uses the ECE3.h library for IR sensor reading and encoder access.

## Board:
- RED LaunchPad w/ msp432p401r EMT (48MHz)

## Calibrations (Control Equations)

### IR Normalization
Each IR sensor reading is mapped from calibrated min/max to a normalized 0–1000 scale:

$$
v_i = 1000 \cdot \frac{s_i - \text{min}_i}{\text{max}_i - \text{min}_i}
$$

### Lateral Error Calculation
Weighted sum of sensors produces the line-following error:

$$
e = \frac{1}{8} \sum_{i=1}^{8} w_i v_i
$$

### PID Steering
The steering correction is:

$$
u = K_p e + K_d (e - e_{\text{prev}})
$$

Motor speeds are:

$$
v_L = \text{baseSpeed} - u,\quad v_R = \text{baseSpeed} + u
$$

