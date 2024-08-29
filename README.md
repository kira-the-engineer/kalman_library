## About

The purpose of this project is to create a standalone library of reusable functions to create n-dimensional linear and non-linear kalman filters that run on microcontrollers. The goal is to make this library platform agnostic- with the 
only requirement being that the microcontrollers running this library need to be able to utilize the cstdlib and Eigen.

The creation of this library was inspired by issues with GPS accuracy in my [capstone project](https://github.com/OSURoboticsClub/Rover_RDF_Capstone). The goal was to fuse measurements from an IMU and GNSS data to estimate the "pose" of the robot in the field. The inspiration for this came from the paper: [*GPS-IMU Sensor Fusion for Reliable Autonomous Vehicle Position Estimation*](https://arxiv.org/pdf/2405.08119). Since I've graduated, the scope of this project has changed to be more general in nature. Instead of being specifically for the OSU Mars Rover team, I plan on using this for general robotics and hobbyist projects.

## Implementation

Current implementation of this library is being tested on an Adafruit Feather with an ATSAMD21 ARM Cortex MO micrcontroller on board. All development is being done using PlatformIO. The current libraries used for this project are the [ArxTypeTraits](https://github.com/hideakitai/ArxTypeTraits) library and Bolder Flight's port of [Eigen](https://github.com/bolderflight/eigen). Challenges with implementation include scaling the size of the matricies used for the Kalman Filters while maintaining good space and runtime complexity and finding ways to test and ensure accuracy of the filters.

## To Do List (non-exhaustive)
- [ ] Test the Linear KF code with input from real, linear sensor data
- [ ] Write the sigma point functions for the Unscented Kalman Filter
- [ ] Write class for creating Unscented Kalman Filters
- [ ] Create a way for KF output to be graphed (possibly using Python- or whatever the C++ equivalent of Matplotlib is- this will not run on the microcontroller itself!!)
- [ ] Figure out how to benchmark runtime of code
- [ ] Review math for Extended Kalman Filter
- [ ] Write class for EKF
- [ ] Test EKF and UKF both on fusing GPS and IMU
- [ ] Determine a way to test the accuracy of the filters
