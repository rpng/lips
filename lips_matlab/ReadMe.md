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
4. Copy files saved in the "lins_simulator/output/" to a subfolder
5. In your ROS workspace one can launch the node to publish this data
    * Example command: `rosrun lins_simulator pubSimulation <path_to_data>`
    * Can create a launch file to edit the noise values
    * Publishes true pose and noisy measurements
6. Can record a rosbag of the data, or play it normally


## Floorplan File Format

* Each plane is made of a 2D line between the start and end point
* Note that this is in 2D and is later extruded into 3D
* start_x, start_y, end_x, end_y


## Trajectory File Format

* A trajectory is in 3D in respect to the origin of the floorplan
* The trajectory units are the same as the floorplan's
* We apply the rotations as follows:
* rot_FRAMEtoG = rotz(yaw)*roty(pitch)*rotx(roll)
* The user needs to specify waypoints at a high enough frequency that the spline works
* x,y,z,yaw,pitch,roll

