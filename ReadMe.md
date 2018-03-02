# LiDAR-Inertial 3D Plane Simulator

This repository has a LiDAR-inertial 3D plane simulator in it that allows for custom trajectory through 3D enviroments to be created, and a sensor suite to be sent through it at a given rate.
The simplest way to get started is to clone this repository into your ROS workspace, and play around with the example datasets.


## Package Descriptions

* **lips_comm**: This package has the custom message files that the simulator should publish.
* **lips_matlab**: Contains MATLAB scripts to generate simulated lidar point clouds moving through a 3D environment.
* **lips_simulator**: This package has a publisher node that takes the MATLAB exported files and publish on ROS.


## Screenshots / Pictures

![example lidar run](pictures/example_run.png)

 
