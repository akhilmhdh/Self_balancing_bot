# Self_balancing_bot
An arduino powered two wheeled self balancing bot.
<br>It uses PID algorithm to have an optimised error free movement.
# Getting Started
## Softwares
- Arduino IDE
## Hardwares
- Arduino uno or any other versions
- Two DC motors and height focused body
- IMU sensor(mine : mpu6050)
- A pwm supported motor driver
## Details
The body should have a sufficient height. The pid algo uses PWM to control the motors.I have used imu sensor to obtain error free in angle calculations.Used JEFFs mpu library to obtain the quaternion outputs.The PID constants Kp,Ki,Kd can vary with the ody. It should be noted that each body has varying moment of inertia so varying constants. Soon auto tuning pid will be developed. 
