# Allan Variance ROS2
## ROS2 package which loads a rosbag of IMU data and computes Allan Variance parameters
The purpose of this tool is to read a long sequence of IMU data and compute the Angle Random Walk (ARW), Bias Instability and Gyro Random Walk for the gyroscope as well as Velocity Random Walk (VRW), Bias Instability and Accel Random Walk for the accelerometer.

While there are many open source tools which do the same thing, this package has the following features:

- Fully ROS compatable. Simply record a `rosbag` and provide it as input. No conversion required.
- Written in C++ making use of rosbag::View means the `rosbag` is processed at maximum speed. No need to play back the bag file.

## Author

[Russell Buchanan](https://raabuchanan.com/)


## References

- [Indirect Kalman Filter for 3D Attitude Estimation, Trawny & Roumeliotis](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf)
- [An introduction to inertial navigation, Oliver Woodman](https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf) 
- [Characterization of Errors and Noises in MEMS Inertial Sensors Using Allan Variance Method, Leslie Barreda Pupo](https://upcommons.upc.edu/bitstream/handle/2117/103849/MScLeslieB.pdf?sequence=1&isAllowed=y)
- [Kalibr IMU Noise Documentation](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
