#ifndef PARSELINES_H
#define PARSELINES_H

#include <iostream>
#include <fstream>
#include <sstream>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lips_comm/PlaneMeasurement.h"
#include "lips_comm/PlaneMeasurementArray.h"

#include "quat_ops.h"


/**
 * \brief Given a string line, this will create a ROS pose object from the simulated data format
 */
geometry_msgs::PoseStamped parsePoseLine(ros::Time basetime, std::string frame, std::string line) {
    // Temp variables
    double timestamp = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    // Loop variables
    int i = 0;
    std::istringstream s(line);
    std::string field;
    // Loop through this line
    while (getline(s,field,',')) {
        // column 1: timestamp
        if(i==0) {
            timestamp = std::atof(field.c_str());
        }
        // column 2: x position
        else if(i==1) {
            x = std::atof(field.c_str());
        }
        // column 3: y position
        else if(i==2) {
            y = std::atof(field.c_str());
        }
        // column 4: z position
        else if(i==3) {
            z = std::atof(field.c_str());
        }
        // column 5: global roll
        else if(i==4) {
            roll = std::atof(field.c_str());
        }
        // column 6: global pitch
        else if(i==5) {
            pitch = std::atof(field.c_str());
        }
        // column 7: global yaw
        else if(i==6) {
            yaw = std::atof(field.c_str());
        }
        i++;
    }
    // Create the pose!
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time(basetime.toSec()+timestamp);
    pose.header.frame_id = frame;
    // Pose position
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    // Pose oriention from RPY gives R_ItoG transpose it to get R_GtoI
    Eigen::AngleAxisd xrot(roll,Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yrot(pitch,Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd zrot(yaw,Eigen::Vector3d::UnitZ());
    //Eigen::Quaterniond quat(xrot*yrot*zrot);
    Eigen::Matrix3d rot = zrot.toRotationMatrix()*yrot.toRotationMatrix()*xrot.toRotationMatrix();
    Eigen::Vector4d quat = rot_2_quat(rot.transpose());
    pose.pose.orientation.x = quat(0);
    pose.pose.orientation.y = quat(1);
    pose.pose.orientation.z = quat(2);
    pose.pose.orientation.w = quat(3);
    // Return!
    return pose;
}



/**
 * \brief Given a string line, this will create an IMU measurement from the simulated data format
 */
sensor_msgs::Imu parseImuMeasurementLine(ros::Time basetime, std::string frame, std::string line) {
    // Temp variables
    double timestamp = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;
    // Loop variables
    int i = 0;
    std::istringstream s(line);
    std::string field;
    // Loop through this line
    while (getline(s,field,',')) {
        // column 1: timestamp
        if(i==0) {
            timestamp = std::atof(field.c_str());
        }
        // column 2: x acceleration
        else if(i==1) {
            ax = std::atof(field.c_str());
        }
        // column 3: y acceleration
        else if(i==2) {
            ay = std::atof(field.c_str());
        }
        // column 4: z acceleration
        else if(i==3) {
            az = std::atof(field.c_str());
        }
        // column 5: x angular (roll)
        else if(i==4) {
            wx = std::atof(field.c_str());
        }
        // column 6: y angular (pitch)
        else if(i==5) {
            wy = std::atof(field.c_str());
        }
        // column 7: z angular (yaw)
        else if(i==6) {
            wz = std::atof(field.c_str());
        }
        i++;
    }
    // Create the measurement!
    sensor_msgs::Imu meas;
    meas.header.stamp = ros::Time(basetime.toSec()+timestamp);
    meas.header.frame_id = frame;
    // Linear acceleration
    meas.linear_acceleration.x = ax;
    meas.linear_acceleration.y = ay;
    meas.linear_acceleration.z = az;
    // Angular velocity
    meas.angular_velocity.x = wx;
    meas.angular_velocity.y = wy;
    meas.angular_velocity.z = wz;
    // Return!
    return meas;
}


/**
 * \brief Given a string line, this will create a plane measurement array from the simulated data format
 */
lips_comm::PlaneMeasurementArray parsePlaneMeasurementLine(ros::Time basetime, std::string frame, std::string lineMEAS, std::string lineIDS) {
    // Temp variables
    double timestamp = 0.0;
    int planeid = 0;
    pcl::PointXYZ pt;
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> rawdata;
    // Loop variables
    int i = 0;
    std::istringstream sMEAS(lineMEAS);
    std::istringstream sIDS(lineIDS);
    std::string fieldMEAS;
    std::string fieldIDS;
    // We skip the first field in both, they should be the timestamps
    getline(sMEAS,fieldMEAS,',');
    getline(sIDS,fieldIDS,',');
    // Check that they are both the same time
    if(std::atof(fieldMEAS.c_str()) != std::atof(fieldIDS.c_str())) {
        ROS_ERROR("ERROR: Invalid timestamp matches between lidar simulation files %.4f vs. %.4f",std::atof(fieldMEAS.c_str()),std::atof(fieldIDS.c_str()));
        std::exit(EXIT_FAILURE);
    }
    timestamp = std::atof(fieldMEAS.c_str());
    // Loop through this line
    while (getline(sMEAS,fieldMEAS,',')) {
        // column 1: we should read what ID this point is, and its x value
        if(i==0) {
            getline(sIDS,fieldIDS,',');
            planeid = std::atoi(fieldIDS.c_str());
            pt.x = std::stof(fieldMEAS.c_str());
        }
        // column 2: y position of point
        else if(i==1) {
            pt.y = std::stof(fieldMEAS.c_str());
        }
        // column 3: z position of point (note we reset the i=0 for next iteration)
        else if(i==2) {
            // Get the z position
            pt.z = std::stof(fieldMEAS.c_str());
            // Reset, and append to our map
            if(rawdata.find(planeid) != rawdata.end()) {
                rawdata[planeid]->push_back(pt);
            }
            // If the current plane is not found, then we should create it!
            else {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
                cloud->push_back(pt);
                rawdata.insert(std::pair<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>(planeid, cloud));
            }
            i = -1;
        }
        i++;
    }

    // Convert the hashmap into a vector of the planes
    std::vector<lips_comm::PlaneMeasurement> planes;
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it;
    for (it = rawdata.begin(); it!=rawdata.end(); ++it) {
        lips_comm::PlaneMeasurement plane;
        plane.id = it->first;
        pcl::toROSMsg(*it->second, plane.points);
        planes.push_back(plane);
    }

    // Debug info
    //ROS_INFO("LOAD: Loaded %d planes at time %.4f",(int)planes.size(),timestamp);

    // Create the measurement!
    lips_comm::PlaneMeasurementArray meas;
    meas.header.stamp = ros::Time(basetime.toSec()+timestamp);
    meas.header.frame_id = frame;
    // Linear acceleration
    meas.planes = planes;
    // Return!
    return meas;
}







#endif  //#ifndef PARSELINES_H
