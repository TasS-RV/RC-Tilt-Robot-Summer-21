# RC-tilt-robot-Summer-21
Repository of the Arduino Code for Robot controlled using Arduino Microcontroller, worked on during Summer 2021 period. Predominantly controlled by tilt angle.

1)It uses an MPU 6050 Inertial Measurement Unit (IMU) to measure the Tilt angle of the sensor, through resolution of the gravitational acceleration in x and y axes.
2)This control is completed by a couple of joysticks, where the analog input is used to control the motion of different parts of the mechanical claw/ 
Additional buttons on the remote are used as toggle switches between driving and claw modes: respectively, when the robot is in motion, and when the robot is picking items up as undesired tilt of the IMU can lead to sudden motion and misalignment of claw with items to pick up.

3)All code is written in Arduino Software using C++ code along with various external libraries for functionality of the servos, LCD display, 433Mhz radio transmitter and receiver as well as for the MPU 6050 IMU.


![IMG_20210811_224448](https://user-images.githubusercontent.com/93861976/160678906-1b159c14-475e-4d31-9239-e24250f950eb.jpg)




For working demo of the robot please view the files at: https://drive.google.com/drive/folders/1XXeXPRURs9xHQBbBjRDghO9fq5H1shfX?usp=sharing
Please note that the 'Process' folder contains the final code files used in the robots for the demo, for both the Uno AVR board powering the Transmitter as well as the main Robot itself.

**Interpretting the File in the Repo:**
The 'Main Code' contains the progressive code as each stage of the robot was developed. For instance, '1 Versions - Tilt Steer and Claw Robot' contains .ino files used in the main robot progressing from forwards and backwards motion working, to rotation (left and right motion), all the way until configuring and calibrating angles for the stepper motors and servos of the clae to work. '2 receiver' contains the pure receiver code in the mian robot, while '3 Transmitter' contains purely the code in the Transmission circuit.

