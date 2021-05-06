# Quadrotor UAV
# author: Jackson Qin

This project is about Quadrotor-UAV based on ARM MCU.My graduation project is design a UAV that used PID contorl algorithms and electronic knowledge.I'm a electornic engineering stduent.

This design uses the STM32F407VGT6 based on the ARM Cortex-M4 core as the main control chip of the quadrotor. The ICM20602 six-axis sensor, the AK8975 electronic compass, and the SPL06-001 barometer are selected to obtain the attitude data, and the original sensor data is filtered through the complementary filtering algorithm. Use quaternion method and Euler angle method to calculate attitude. The PID algorithm commonly used in control theory is used to control the motor speed, and the voltage of the motor is controlled by setting different PWM duty ratios to change the motor speed to achieve different flight attitudes of the quadcopter. This design expands the image sensor to collect road image data, and uses the Raspberry Pi OpenCV algorithm platform to process the image data to obtain the center line fitting deviation data, so as to realize the visual navigation function of the quadrotor, so that it can fly along the given cruise line.
