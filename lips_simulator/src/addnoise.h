#ifndef ADDNOISE_H
#define ADDNOISE_H

#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "lips_comm/PlaneMeasurement.h"
#include "lips_comm/PlaneMeasurementArray.h"


// Random device class instance, source of 'true' randomness for initializing random seed
std::random_device rd;

// Mersenne twister PRNG, initialized with seed from previous random device instance
std::mt19937 gen(rd());
//std::mt19937 gen(1234);

// Our normal distribution that we will sample values from
std::normal_distribution<double> w(0,1);


/**
 * \brief Add noise to our IMU measurement
 */
sensor_msgs::Imu addnoiseIMUMeasurement(sensor_msgs::Imu meas, int imurate,
                                        double gyroscope_noise_density, double accelerometer_noise_density,
                                        double gyroscope_random_walk, double accelerometer_random_walk,
                                        Eigen::Vector3d &gyroscope_bias, Eigen::Vector3d &accelerometer_bias) {
    // Our IMU delta time
    double deltat = 1.0/imurate;

    // Our new measurement
    sensor_msgs::Imu meas_noisy;
    meas_noisy.header = meas.header;

    // Add noise to each measurement direction
    meas_noisy.angular_velocity.x = meas.angular_velocity.x + gyroscope_bias(0) + gyroscope_noise_density*1/std::sqrt(deltat)*w(gen);
    meas_noisy.angular_velocity.y = meas.angular_velocity.y + gyroscope_bias(1) + gyroscope_noise_density*1/std::sqrt(deltat)*w(gen);
    meas_noisy.angular_velocity.z = meas.angular_velocity.z + gyroscope_bias(2) + gyroscope_noise_density*1/std::sqrt(deltat)*w(gen);
    meas_noisy.linear_acceleration.x = meas.linear_acceleration.x + accelerometer_bias(0) + accelerometer_noise_density*1/std::sqrt(deltat)*w(gen);
    meas_noisy.linear_acceleration.y = meas.linear_acceleration.y + accelerometer_bias(1) + accelerometer_noise_density*1/std::sqrt(deltat)*w(gen);
    meas_noisy.linear_acceleration.z = meas.linear_acceleration.z + accelerometer_bias(2) + accelerometer_noise_density*1/std::sqrt(deltat)*w(gen);

    // Move the biases forward in time
    gyroscope_bias(0) += gyroscope_random_walk*std::sqrt(deltat)*w(gen);
    gyroscope_bias(1) += gyroscope_random_walk*std::sqrt(deltat)*w(gen);
    gyroscope_bias(2) += gyroscope_random_walk*std::sqrt(deltat)*w(gen);
    accelerometer_bias(0) += accelerometer_random_walk*std::sqrt(deltat)*w(gen);
    accelerometer_bias(1) += accelerometer_random_walk*std::sqrt(deltat)*w(gen);
    accelerometer_bias(2) += accelerometer_random_walk*std::sqrt(deltat)*w(gen);

    // Print bias
    //std::cout << "GYRO BIAS = " << gyroscope_bias.transpose() << std::endl;
    //std::cout << "ACCL BIAS = " << accelerometer_bias.transpose() << std::endl;

    // Return!
    return meas_noisy;

}


/**
 * \brief Given a string line, this will create a ROS pose object from the simulated data format
 */
lips_comm::PlaneMeasurementArray addnoisePlaneMeasurement(lips_comm::PlaneMeasurementArray meas, double sigmap) {

    // Our new measurement
    lips_comm::PlaneMeasurementArray meas_noisy;
    meas_noisy.header = meas.header;

    // Loop through each plane
    for(size_t i=0; i<meas.planes.size(); i++) {

        // Our noisy plane
        lips_comm::PlaneMeasurement plane_noisy;
        plane_noisy.id = meas.planes.at(i).id;

        // Convert point cloud to something we can use
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(meas.planes.at(i).points, *cloud);

        // For each point, we want to add some noise
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noisy(new pcl::PointCloud<pcl::PointXYZ>);
        for(size_t j=0; j<cloud->points.size(); j++) {
            // Create our noisy point
            pcl::PointXYZ pointnoisy;
            pointnoisy.x = cloud->at(j).x + sigmap*w(gen);
            pointnoisy.y = cloud->at(j).y + sigmap*w(gen);
            pointnoisy.z = cloud->at(j).z + sigmap*w(gen);
            cloud_noisy->push_back(pointnoisy);
            // Debug info
            //ROS_INFO("pt %d => (%.4f,%.4f,%.4f) with noise is (%.4f,%.4f,%.4f) %.4f",(int)j,cloud->at(j).x,cloud->at(j).y,cloud->at(j).z,pointnoisy.x,pointnoisy.y,pointnoisy.z,sigmap*w(gen));
        }

        // Append to our noisy measurement
        pcl::toROSMsg(*cloud_noisy, plane_noisy.points);
        meas_noisy.planes.push_back(plane_noisy);
    }

    // Return!
    return meas_noisy;
}


#endif  //#ifndef ADDNOISE_H
