# Self_balancing_bot
An arduino powered two wheeled self balancing bot.
<br>It uses PID algorithm to have an optimised error free movement.
# Getting Started
## Softwares
- Arduino IDE
## Hardwares
- Arduino uno or any other versions
- Two DC motors and height focused body
- A pwm supported motor driver
## Details
The body should have a sufficient height. The pid algo uses PWM to control the motors. The PID constants Kp,Ki,Kd can vary with the ody. It should be noted that each body has varying moment of inertia so varying constants. Soon auto tuning pid will be developed. 
