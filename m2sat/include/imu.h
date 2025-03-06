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

static const Quaterniond q_b2imu(std::sqrt(0.5), 0, std::sqrt(0.5), 0); // imu is rotated around body frame y axis by positive 90 degrees which is this quaternion

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

        tele->q_b2i =  q_b2imu.inverse()*tele->q_b2i; 
        
        // body angular velocity
        tele->omega_b2i_B.x() = angularRate.x; 
        tele->omega_b2i_B.y() = angularRate.y; 
        tele->omega_b2i_B.z() = angularRate.z;    
        // apply rotation to map imu frame to body frame on ang velocity
        tele->omega_b2i_B = quat_rotate(q_b2imu.inverse(), tele->omega_b2i_B);
    }

    Vector3d quat_rotate(const Quaterniond& q, const Vector3d& v)
    {
        Quaterniond v_quat(0, v.x(), v.y(), v.z());  // Pure quaternion [0; v]
        Quaterniond rotated = q * v_quat * q.conjugate();  // q * [0; v] * q^*
        return rotated.vec();  // Extract the vector part (x, y, z)
    }

    
};

void ConnectAndConfigureIMU(imu_data_vn_format_t * imu_data, std::mutex * imu_mutex);