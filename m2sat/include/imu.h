#pragma once

#include <iostream>
#include <mutex>
#include <Eigen/Geometry> 
#include <Eigen/Dense>

#include "vn/sensors.h" // Include this header file to get access to VectorNav sensors.
#include "vn/thread.h" // We need this file for our sleep function.

#include "telemetry.h"
// #include <types.h>

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

#define VN_IMU_SERIAL_PORT "/dev/ttyUSB0"

/* IF THE  VN_IMU_OUTPUT_FORMAT IS CHANGED, THEN THE PARSE FUNCTION IN imu.c MUST ALSO BE CHANGED*/
static const  vn::protocol::uart::AsciiAsync imu_connection_mode = vn::protocol::uart::VNQMR;

/**
 * The IMU is mounted 90 degrees clockwise about the z axis relative to our body coordinate system
 * This rotation converts from imu frame to the body frame.
 */
const Eigen::Matrix3d imu2body = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
const Eigen::Quaterniond imu2body_quat(imu2body);

struct imu_data_vn_format_t
{
    vec4f quaternion; // pose
    vec3f magnetic; // ignored for now
    vec3f acceleration; //m/s^2
    vec3f angularRate; // rad/s

    bool valid_data = false; // set to true at the first instance of data received, prevents controller from running without data

    void PullMeasurement(telemetry_t * tele) // fill our telemetry packet 
    {   
        // convert to Eigen quaternion
        tele->q_b2i.w() = quaternion.w;
        tele->q_b2i.x() = quaternion.x;
        tele->q_b2i.y() = quaternion.y;
        tele->q_b2i.z() = quaternion.z;
        // tele->imu.q = imu2body_quat * tele->imu.q; // rotate

        // // convert to euler here 
        // tele->imu.euler = tele->imu.q.toRotationMatrix().eulerAngles(0, 1, 2); // todo check this
        // tele->imu.euler = imu2body * tele->imu.euler; // rotate

        // /* Note, the imu is mounted with the positive y of the imu = -x of the body frame. positive x imu = positive y body */
        // // accelerations
        // tele->imu.accel.x() = acceleration.x;
        // tele->imu.accel.y() = acceleration.y; 
        // tele->imu.accel.z() = -acceleration.z; 

        // body angular velocity
        tele->omega_b2i_B.x() = angularRate.x; 
        tele->omega_b2i_B.y() = angularRate.y; 
        tele->omega_b2i_B.z() = angularRate.z;    
    }

    
};

void ConnectAndConfigureIMU(imu_data_vn_format_t * imu_data, std::mutex * imu_mutex);