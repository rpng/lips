# lips_simulator

This has a bunch of matlab scripts that allows for the simulation of a 3D LIDAR within a planar enviroment.
A user can specify a 2D floorplan and 3D trajectory through that space.
This floorplan is "extruded" into 3D by specifying the wall height of the floor, this allows for simple map creation.
From here the trajectory is interpolated using matlab's spline functions.
At each timestep a set of "rays" from the LIDAR center are shot out and ray-polygon intersection is performed for each ray plane combination.
From all the valid intersection, we further enforce that a ray hits the plane that is closest to the LIDAR center (retaining the occlusion property).
Theses points can now be used in estimation as each point intersection has a corrisponding plane ID (removing the coorespondences problem).

## Running the Simulator

1. First open the matlab folder in matlab
2. Specify the floorplane, trajectory file, and the LIDAR specification
3. Run the "MAINSCRIPT.m" file to generate the IMU, LIDAR, and LIDAR planes clouds
4. Copy files saved in the "lips_simulator/output/" to a subfolder
5. Publish onto ROS using the lips_simulator package


## Floorplan File Format

* Each plane is made of a 2D line between the start and end point
* Note that this is in 2D and is later extruded into 3D
* start_x, start_y, end_x, end_y
* Floorplan should be units of "feet"


## Trajectory File Format

* A trajectory is in 3D in respect to the origin of the floorplan
* We apply the rotations as follows:
* `rot_ItoG = rotz(yaw)*roty(pitch)*rotx(roll)`
* The user needs to specify waypoints at a high enough frequency that the spline works
* time,x,y,z,yaw,pitch,roll
* The waypoints are of the IMU sensor, the LIDAR is calculated using the specified transform

