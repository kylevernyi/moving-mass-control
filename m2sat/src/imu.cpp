#include <thread>
#include <chrono>

#include "imu.h"
#include "vn/thread.h" // We need this file for our sleep function.
#include "vn/sensors.h" // Include this header file to get access to VectorNav sensors.

// I dont normally like namespaces but we dont use the VN lib too much 
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

static VnSensor vs; // this needs to be defined outside of a temporary function stack like the ConnectAndConfigure, since it needs to stay alive
static std::mutex * imu_mutex_;

#define VN_IMU_BAUD_RATE 115200
#define VN_IMU_UPDATE_HZ 100

void asciiAsyncMessageReceived(void* imu_struct, Packet& p, size_t index);

void ConnectAndConfigureIMU(imu_data_vn_format_t * imu_data, std::mutex * imu_mutex)
{
    imu_mutex_ = imu_mutex;
	
    vs.connect(VN_IMU_SERIAL_PORT,   VN_IMU_BAUD_RATE); // connect to sensor
	
    /* Heading 2D */
    FilterBasicControlRegister reg = vs.readFilterBasicControl();
    reg.magMode = MAGNETICMODE_2D; // magnetometer only effects heading
    vs.writeFilterBasicControl(reg, 1);

    /* Tell IMU it is rotated 90 degrees about the y axis of the body */
    // The IMU needs UNPLUGGED and REPLUGGED if this matrix changes to apply the change
    const vn::math::mat3f rotationMatrix(
        0.0,  0.0f,  -1.0f, 
		0.0f, 1.0f,  0.0f, 
		1.0f,  0.0f, 0.0f);
	vs.writeReferenceFrameRotation(rotationMatrix, true);
	vs.writeSettings(true); 

    /* Indoor mode for heading */
    VpeBasicControlRegister vpeReg = vs.readVpeBasicControl();
    vpeReg.headingMode = HEADINGMODE_INDOOR;
    vs.writeVpeBasicControl(vpeReg);

    vs.writeAsyncDataOutputFrequency(VN_IMU_UPDATE_HZ); // set output frequency
	uint32_t newHz = vs.readAsyncDataOutputFrequency();

    vs.writeAsyncDataOutputType(imu_connection_mode); // set data output format, false = dont wait for reply
    vs.registerAsyncPacketReceivedHandler((void *) imu_data, asciiAsyncMessageReceived); // register callback function

}

void tare_heading()
{
    vs.tare(1);
}

void asciiAsyncMessageReceived(void* imu_struct, Packet& p, size_t index)
{
    if (!imu_mutex_) // make sure our mutex exists
        return;

    if(p.determineAsciiAsyncType() == imu_connection_mode) { // just double check we received the right packet type
        
        imu_data_vn_format_t * imu_output_ptr = static_cast<imu_data_vn_format_t*>(imu_struct); // casting into usable struct
        
        // must lock 
        if (imu_mutex_->try_lock()) { 
            /* VERY IMPORTANT: IF WE CHANGE THE OUTPUT FORMAT, THIS PARSE MUST ALSO CHANGE */
            // p.parseVNICM(&imu_ptr->yawPitchRoll, &imu_ptr->magnetic, &imu_ptr->acceleration, &imu_ptr->angularRate); // copy data into our structs
            imu_data_vn_format_t imu_data;
            imu_data_vn_format_t * imu_ptr = &imu_data;

            // p.parseVNQMR(&imu_ptr->quaternion, &imu_ptr->magnetic, &imu_ptr->acceleration, &imu_ptr->angularRate);
            p.parseVNQTR(&imu_ptr->quaternion, &imu_ptr->angularRate);

            imu_output_ptr->quaternion = imu_ptr->quaternion;
            // imu_output_ptr->magnetic = imu_ptr->magnetic;
            // imu_output_ptr->acceleration = imu_ptr->acceleration;
            imu_output_ptr->angularRate = imu_ptr->angularRate;


            imu_output_ptr->valid_data = true;
            imu_mutex_->unlock(); 
        }

		// std::cout << "got imu" << std::endl;
    }
}