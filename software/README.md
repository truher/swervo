Swervo Software
===============

The software mostly follows the WPILIB swerve example, with a few extra bits:


* The gyro and compass components of the [IMU](https://www.adafruit.com/product/4517) each have their own I2C interface class, and the signals produced are combined using a simple Kalman filter.
* The [Parallax 360](https://www.parallax.com/product/parallax-feedback-360-high-speed-servo/) has a trivial driver subclass of DutyCycleController.
