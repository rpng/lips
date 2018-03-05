#include <cmath>
#include <vector>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <random>

#include <tf/tf.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>


#include "lips_comm/PlaneMeasurement.h"
#include "lips_comm/PlaneMeasurementArray.h"

#include "parselines.h"
#include "addnoise.h"

// Simulation data from file
int imurate;
int lidarrate;
std::vector<geometry_msgs::PoseStamped> poses_imu;
std::vector<geometry_msgs::PoseStamped> poses_lidar;
std::vector<sensor_msgs::Imu> measurements_imu;
std::vector<sensor_msgs::Imu> measurements_imu_true;
std::vector<lips_comm::PlaneMeasurementArray> measurements_lidar;
std::vector<lips_comm::PlaneMeasurementArray> measurements_lidar_true;


// Current index in each data vector
size_t index_pi = 0;
size_t index_pl = 0;
size_t index_mi = 0;
size_t index_ml = 0;


// Publishers
tf::TransformBroadcaster* mTfBr;
ros::Publisher pubPoseIMU;
ros::Publisher pubPoseLIDAR;
ros::Publisher pubPathLIDAR;
ros::Publisher pubMeasurementIMU;
ros::Publisher pubMeasurementLIDAR;
ros::Publisher pubMeasurementCloudLIDAR;

double gyroscope_noise_density; //rad/s/sqrt(hz)
double accelerometer_noise_density; //m/s^2/sqrt(hz)
double gyroscope_random_walk; //rad/s^2/sqrt(hz)
double accelerometer_random_walk; //m/s^3/sqrt(hz)
Eigen::Vector3d gyroscope_bias;
Eigen::Vector3d accelerometer_bias;
double sigma_lidar_pts; //meters


// Path variables needed
unsigned int poses_seq_lidar = 0;
vector<geometry_msgs::PoseStamped> posespath_lidar;


/**
 * \brief This will try to load all the key files in a given simulation folder
 * \brief It will error if it is missing a file in the given path
 *
 * @param path Path to the folder that has the simulation txt files
 */
void load_files(std::string path) {

    // Base ROS time we will add all the simulated times too
    // This is fixed since we want each simulation to at the same time with different noise
    ros::Time basetime = ros::Time(1519060444);

    // Master file stream object
    std::ifstream file01;
    std::ifstream file02;
    std::string line01;
    std::string line02;

    //======================================================================
    // READ THE CONFIG RATES FILE
    file01.open(path+"config_rates.txt");
    if (!file01) {
        ROS_ERROR("ERROR: Unable to open file...");
        ROS_ERROR("ERROR: %s",(path+"config_rates.txt").c_str());
        std::exit(EXIT_FAILURE);
    }
    while(std::getline(file01, line01) && ros::ok()) {
        std::string field;
        std::istringstream s(line01);
        // Parse IMU rate
        std::getline(s,field,',');
        imurate = std::atoi(field.c_str());
        // Parse LIDAR rate
        std::getline(s,field,',');
        lidarrate = std::atoi(field.c_str());
        // That all we need to read, break out
        break;
    }
    file01.close();


    //======================================================================
    // READ THE TRUE IMU POSE FILE
    file01.open(path+"true_poseimu.txt");
    if (!file01) {
        ROS_ERROR("ERROR: Unable to open file...");
        ROS_ERROR("ERROR: %s",(path+"true_poseimu.txt").c_str());
        std::exit(EXIT_FAILURE);
    }
    while(std::getline(file01, line01) && ros::ok()) {
        // Get the pose for this line
        geometry_msgs::PoseStamped pose;
        pose = parsePoseLine(basetime, "global", line01);
        // Append it to our vector of true poses
        poses_imu.push_back(pose);
    }
    file01.close();

    //======================================================================
    // READ THE TRUE IMU MEASUREMENTS FILE
    file01.open(path+"true_measimu.txt");
    if (!file01) {
        ROS_ERROR("ERROR: Unable to open file...");
        ROS_ERROR("ERROR: %s",(path+"true_measimu.txt").c_str());
        std::exit(EXIT_FAILURE);
    }
    while(std::getline(file01, line01) && ros::ok()) {
        // Get the measurement for this line
        sensor_msgs::Imu meas_true;
        meas_true = parseImuMeasurementLine(basetime, "imu", line01);
        // Append it to our vector of true measurements
        measurements_imu_true.push_back(meas_true);
        // Get a noisy measurement
        sensor_msgs::Imu meas_noisy;
        meas_noisy = addnoiseIMUMeasurement(meas_true,imurate, gyroscope_noise_density, accelerometer_noise_density,
                                            gyroscope_random_walk, accelerometer_random_walk, gyroscope_bias, accelerometer_bias);
        // Append it to our vector of measurements
        measurements_imu.push_back(meas_noisy);
    }
    file01.close();


    //======================================================================
    // READ THE TRUE LIDAR POSE FILE
    file01.open(path+"true_poselidar.txt");
    if (!file01) {
        ROS_ERROR("ERROR: Unable to open file...");
        ROS_ERROR("ERROR: %s",(path+"true_poselidar.txt").c_str());
        std::exit(EXIT_FAILURE);
    }
    while(std::getline(file01, line01) && ros::ok()) {
        // Get the pose for this line
        geometry_msgs::PoseStamped pose;
        pose = parsePoseLine(basetime, "global", line01);
        // Append it to our vector of true poses
        poses_lidar.push_back(pose);
    }
    file01.close();


    //======================================================================
    // READ THE TRUE IMU MEASUREMENTS FILE
    file01.open(path+"true_measplanes.txt");
    if (!file01) {
        ROS_ERROR("ERROR: Unable to open file...");
        ROS_ERROR("ERROR: %s",(path+"true_measplanes.txt").c_str());
        std::exit(EXIT_FAILURE);
    }
    file02.open(path+"true_idsplanes.txt");
    if (!file02) {
        ROS_ERROR("ERROR: Unable to open file...");
        ROS_ERROR("ERROR: %s",(path+"true_idsplanes.txt").c_str());
        std::exit(EXIT_FAILURE);
    }
    while(std::getline(file01, line01) && std::getline(file02, line02) && ros::ok()) {
        // Get the measurement for this line
        lips_comm::PlaneMeasurementArray meas_true;
        meas_true = parsePlaneMeasurementLine(basetime, "lidar", line01, line02);
        // Append it to our vector of true measurements
        measurements_lidar_true.push_back(meas_true);
        // Get a noisy measurement
        lips_comm::PlaneMeasurementArray meas_noisy;
        meas_noisy = addnoisePlaneMeasurement(meas_true,sigma_lidar_pts);
        // Append it to our vector of measurements
        measurements_lidar.push_back(meas_noisy);
    }
    file01.close();
    file02.close();


    // Debug information
    ROS_INFO("LOAD: %d true IMU poses",(int)poses_imu.size());
    ROS_INFO("LOAD: %d true IMU measurements",(int)measurements_imu_true.size());
    ROS_INFO("LOAD: %d meas IMU measurements",(int)measurements_imu.size());
    ROS_INFO("LOAD: %d true LIDAR poses",(int)poses_lidar.size());
    ROS_INFO("LOAD: %d true LIDAR measurements",(int)measurements_lidar_true.size());
    ROS_INFO("LOAD: %d meas LIDAR measurements",(int)measurements_lidar.size());


    // Check that we at least have one measurement of each
    if(poses_imu.empty() || measurements_imu.empty() || poses_lidar.empty() || measurements_lidar.empty()) {
        ROS_ERROR("ERROR: Need at least one of each type of measurement...cannot continue.");
        std::exit(EXIT_FAILURE);
    }


}



/**
 * \brief This performs the setup for all the ROS publishers needed
 */
void setup_publishers(ros::NodeHandle nh) {

    // ROS TF
    mTfBr = new tf::TransformBroadcaster();

    // True pose information
    pubPoseIMU = nh.advertise<geometry_msgs::PoseStamped>("/lips_sim/truepose_imu", 2);
    ROS_INFO("Publishing: %s", pubPoseIMU.getTopic().c_str());
    pubPoseLIDAR = nh.advertise<geometry_msgs::PoseStamped>("/lips_sim/truepose_lidar", 2);
    ROS_INFO("Publishing: %s", pubPoseLIDAR.getTopic().c_str());

    // True pose path
    pubPathLIDAR = nh.advertise<nav_msgs::Path>("/lips_sim/truepath_lidar", 2);
    ROS_INFO("Publishing: %s", pubPathLIDAR.getTopic().c_str());

    // Measurements
    pubMeasurementIMU = nh.advertise<sensor_msgs::Imu>("/lips_sim/data_imu", 2);
    ROS_INFO("Publishing: %s", pubMeasurementIMU.getTopic().c_str());
    pubMeasurementLIDAR = nh.advertise<lips_comm::PlaneMeasurementArray>("/lips_sim/data_lidar", 2);
    ROS_INFO("Publishing: %s", pubMeasurementLIDAR.getTopic().c_str());

    // Point cloud
    pubMeasurementCloudLIDAR = nh.advertise<sensor_msgs::PointCloud2>("/lips_sim/data_cloud", 2);
    ROS_INFO("Publishing: %s", pubMeasurementCloudLIDAR.getTopic().c_str());

}

/**
 * \brief This will read in config values from the launch file for our sigmas
 */
void setup_config(ros::NodeHandle& nh) {

    // IMU values from sensor sheet and allan-devation chart
    nh.param<double>("gyroscope_noise_density", gyroscope_noise_density, 0.005);
    nh.param<double>("accelerometer_noise_density", accelerometer_noise_density, 0.01);
    nh.param<double>("gyroscope_random_walk", gyroscope_random_walk, 4.0e-06);
    nh.param<double>("accelerometer_random_walk", accelerometer_random_walk, 0.0002);

    // Initial bias parameters
    double g_bias_init;
    double a_bias_init;
    nh.param<double>("gyroscope_bias_init", g_bias_init, 0.01);
    nh.param<double>("accelerometer_bias_init", a_bias_init, 0.05);

    // Store in our Eigen format
    gyroscope_bias(0) = g_bias_init;
    gyroscope_bias(1) = g_bias_init;
    gyroscope_bias(2) = g_bias_init;
    accelerometer_bias(0) = a_bias_init;
    accelerometer_bias(1) = a_bias_init;
    accelerometer_bias(2) = a_bias_init;

    // Lidar normally has 3cm error
    nh.param<double>("sigma_lidar_pts", sigma_lidar_pts, 0.03);

    // Print it all out for our user
    ROS_INFO("======================================");
    ROS_INFO("===== GYROSCOPE NOISE PARAMETERS =====");
    ROS_INFO("======================================");
    ROS_INFO("\tgyro_noise = %.5f",gyroscope_noise_density);
    ROS_INFO("\tgyro_walk = %.8f",gyroscope_random_walk);
    ROS_INFO("\tgyro_init = %.5f",g_bias_init);
    ROS_INFO("======================================");
    ROS_INFO("=== ACCELEROMETER NOISE PARAMETERS ===");
    ROS_INFO("======================================");
    ROS_INFO("\taccel_noise = %.5f",accelerometer_noise_density);
    ROS_INFO("\taccel_walk = %.5f",accelerometer_random_walk);
    ROS_INFO("\taccel_init = %.5f",a_bias_init);
    ROS_INFO("======================================");
    ROS_INFO("======= LIDAR NOISE PARAMETERS =======");
    ROS_INFO("======================================");
    ROS_INFO("\tpoint_noise = %.5f",sigma_lidar_pts);
    ROS_INFO("======================================");

}


/**
 * \brief Convert the plane measurements into a full point cloud that can be used as a measurement
 * Or just for visulization. Will have plane IDs as the intensity values of the XYZI point.
 */
sensor_msgs::PointCloud2 plane_measurement_to_cloud(lips_comm::PlaneMeasurementArray msg) {

    // Cloud where the intensity is what plane it is a part of
    pcl::PointCloud<pcl::PointXYZI> laserCloud;

    // From here we should publish our clouds
    for(size_t i=0; i<msg.planes.size(); i++) {

        // Convert to a pcl cloud
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        pcl::fromROSMsg(msg.planes.at(i).points, temp_cloud);

        // Take this plane, and add it
        for (size_t j = 0; j < temp_cloud.points.size(); j++) {
            pcl::PointXYZI point;
            point.x = temp_cloud.points[j].x;
            point.y = temp_cloud.points[j].y;
            point.z = temp_cloud.points[j].z;
            point.intensity = msg.planes.at(i).id;
            laserCloud.push_back(point);
        }
    }

    // Finally publish our point cloud
    sensor_msgs::PointCloud2 msgOut;
    pcl::toROSMsg(laserCloud, msgOut);
    msgOut.header.stamp = msg.header.stamp;
    msgOut.header.frame_id = msg.header.frame_id;
    return msgOut;
}




/**
 * \brief Will loop through our data, and publish it in realtime
 */
void execute_publishing() {

    // Our total bag time
    double timetotal = poses_imu.at(poses_imu.size()-1).header.stamp.toSec()-poses_imu.at(0).header.stamp.toSec();

    // Debug info
    ROS_INFO("Starting publishing simulated data!!");
    ROS_INFO("Looping at max rate of %d",std::max(imurate,lidarrate));
    ROS_INFO("Total runtime is %.2f seconds\n",timetotal);

    // Our loop rate should be faster then our fastest message type
    ros::Rate loop_rate(std::max(imurate,lidarrate));

    // Loop till we have used up all measurements
    // Also check to make sure that ROS does not want us to stop
    while(ros::ok() && index_pi < poses_imu.size() && index_mi < measurements_imu.size()
          && index_pl < poses_lidar.size() && index_ml < measurements_lidar.size()) {

        // Get what our timestamps are (note we set it to infinity if we are done with that data vector)
        double timepi = (index_pi < poses_imu.size())? poses_imu.at(index_pi).header.stamp.toSec() : std::numeric_limits<double>::max();
        double timemi = (index_mi < measurements_imu.size())? measurements_imu.at(index_mi).header.stamp.toSec() : std::numeric_limits<double>::max();
        double timepl = (index_pl < poses_lidar.size())? poses_lidar.at(index_pl).header.stamp.toSec() : std::numeric_limits<double>::max();
        double timeml = (index_ml < measurements_lidar.size())? measurements_lidar.at(index_ml).header.stamp.toSec() : std::numeric_limits<double>::max();

        // First calculate the current min time
        double timecurr = std::min(timepi,std::min(timemi,std::min(timepl,timeml)));

        // Check to see if the IMU POSE is the min
        if(timepi <= timecurr) {

            // Publish normal pose
            pubPoseIMU.publish(poses_imu.at(index_pi));

            // Publish TF message
            tf::StampedTransform tfPose;
            tfPose.stamp_ = poses_imu.at(index_pi).header.stamp;
            tfPose.frame_id_ = "global";
            tfPose.child_frame_id_ = "imu";
            tf::Quaternion quat(poses_imu.at(index_pi).pose.orientation.x,poses_imu.at(index_pi).pose.orientation.y,poses_imu.at(index_pi).pose.orientation.z,poses_imu.at(index_pi).pose.orientation.w);
            tfPose.setRotation(quat);
            tf::Vector3 orig(poses_imu.at(index_pi).pose.position.x,poses_imu.at(index_pi).pose.position.y,poses_imu.at(index_pi).pose.position.z);
            tfPose.setOrigin(orig);
            mTfBr->sendTransform(tfPose);

            // Move forward in time
            index_pi++;
        }

        // Check to see if the IMU MEASUREMENT is the min
        if(timemi <= timecurr) {
            pubMeasurementIMU.publish(measurements_imu.at(index_mi));
            index_mi++;
        }

        // Check to see if the LIDAR POSE is the min
        if(timepl <= timecurr) {

            // Publish normal pose
            pubPoseLIDAR.publish(poses_lidar.at(index_pl));

            // Publish TF message
            tf::StampedTransform tfPose;
            tfPose.stamp_ = poses_lidar.at(index_pl).header.stamp;
            tfPose.frame_id_ = "global";
            tfPose.child_frame_id_ = "lidar";
            tf::Quaternion quat(poses_lidar.at(index_pl).pose.orientation.x,poses_lidar.at(index_pl).pose.orientation.y,poses_lidar.at(index_pl).pose.orientation.z,poses_lidar.at(index_pl).pose.orientation.w);
            tfPose.setRotation(quat);
            tf::Vector3 orig(poses_lidar.at(index_pl).pose.position.x,poses_lidar.at(index_pl).pose.position.y,poses_lidar.at(index_pl).pose.position.z);
            tfPose.setOrigin(orig);
            mTfBr->sendTransform(tfPose);

            // Publish the path
            posespath_lidar.push_back(poses_lidar.at(index_pl));
            nav_msgs::Path patharr;
            patharr.header.stamp = poses_lidar.at(index_pl).header.stamp;
            patharr.header.seq = poses_seq_lidar++;
            patharr.header.frame_id = "global";
            patharr.poses = posespath_lidar;
            pubPathLIDAR.publish(patharr);

            // Move forward in time
            index_pl++;
        }

        // Check to see if the LIDAR MEASUREMENT is the min
        if(timeml <= timecurr) {
            // Publish our normal measurement
            pubMeasurementLIDAR.publish(measurements_lidar.at(index_ml));

            // Publish our measurement point cloud
            sensor_msgs::PointCloud2 cloud_meas;
            cloud_meas = plane_measurement_to_cloud(measurements_lidar.at(index_ml));
            pubMeasurementCloudLIDAR.publish(cloud_meas);

            // Move forward in time
            index_ml++;
        }

        // Get what our "new" timestamps are
        timepi = (index_pi < poses_imu.size())? poses_imu.at(index_pi).header.stamp.toSec() : std::numeric_limits<double>::max();
        timemi = (index_mi < measurements_imu.size())? measurements_imu.at(index_mi).header.stamp.toSec() : std::numeric_limits<double>::max();
        timepl = (index_pl < poses_lidar.size())? poses_lidar.at(index_pl).header.stamp.toSec() : std::numeric_limits<double>::max();
        timeml = (index_ml < measurements_lidar.size())? measurements_lidar.at(index_ml).header.stamp.toSec() : std::numeric_limits<double>::max();

        // Finally calculate what the "next" timestep will be
        double timenext = std::min(timepi,std::min(timemi,std::min(timepl,timeml)));

        // We are done if we have an infinite time!
        if(timenext == std::numeric_limits<double>::max())
            break;

        // Debug print
        //ROS_INFO("SLEEP DT = %.6f (seconds)",timenext-timecurr);
        double timeelapsed = timecurr-poses_imu.at(0).header.stamp.toSec();
        printf("\r [RUNNING]  Simulation Time: %13.6f   Duration: %.6f / %.6f     \r", timecurr, timeelapsed, timetotal);
        fflush(stdout);

        // Make sure ROS publishes the messages
        ros::spinOnce();

        // Thus we should sleep the amount between our current and the next timestamp
        //ros::Duration(timenext-timecurr).sleep();
        //usleep((uint)(1e6*(timenext-timecurr)));
        loop_rate.sleep();

    }

}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "pubSimulation");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    // Check if there is a path to a dataset
    if(argc < 2) {
        ROS_ERROR("ERROR: Please specify a data folder...");
        ROS_ERROR("Command Example: rosrun lips_simulator pubSimulation <data_folder>");
        return EXIT_FAILURE;
    }

    // Read in config info from the launch file
    setup_config(nhPrivate);

    // Parse the input
    std::string pathFolder = argv[1];

    // Load the simulation data
    load_files(pathFolder);

    // Next up, setup our publishers
    setup_publishers(nh);

    // Finally, lets start the main loop and publish
    execute_publishing();


    // Done!
    return EXIT_SUCCESS;
}
