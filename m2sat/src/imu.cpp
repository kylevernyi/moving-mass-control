#include "imu.h"
// I dont normally like namespaces but we dont use the VN lib too much 


static VnSensor vs;  // this needs to be defined outside of a temporary function stack like the ConnectAndConfigure, since it needs to stay alive
static std::mutex * imu_mutex_;

#include <thread>
#include <chrono>

static const double iir_gain = 0.75;

#define VN_IMU_BAUD_RATE 115200
#define VN_IMU_UPDATE_HZ 10

void asciiAsyncMessageReceived(void* imu_struct, Packet& p, size_t index);

void ConnectAndConfigureIMU(imu_data_vn_format_t * imu_data, std::mutex * imu_mutex)
{
    imu_mutex_ = imu_mutex;
	
    vs.connect(VN_IMU_SERIAL_PORT,   VN_IMU_BAUD_RATE); // connect to sensor
	
    // VN100 is rotated 90 degrees about the z axis, rotate it -90 to align it back
    // const vn::math::mat3f rotationMatrix(1.0,  0.0f,  0.0f, 
    //                                0.0f, 1,  0, 
    //                                 0,  0,  1.0f);
    // vs.writeReferenceFrameRotation(rotationMatrix, true);
    // const vn::math::mat3f reg_value = vs.readReferenceFrameRotation();
    // // std::cout << str(reg_value) << std::endl;
    
    // vs.writeSettings(true); // write and reset required
    // vs.reset(true);
    // std::this_thread::sleep_for(std::chrono::microseconds(3000));
    // vs.connect(VN_IMU_SERIAL_PORT,   VN_IMU_BAUD_RATE); // connect to sensor

    
    vs.writeAsyncDataOutputFrequency(VN_IMU_UPDATE_HZ); // set output frequency
	uint32_t newHz = vs.readAsyncDataOutputFrequency();

    vs.writeAsyncDataOutputType(imu_connection_mode); // set data output format, false = dont wait for reply
    vs.registerAsyncPacketReceivedHandler((void *) imu_data, asciiAsyncMessageReceived); // register callback function




    // exit(-1);
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

            p.parseVNQMR(&imu_ptr->quaternion, &imu_ptr->magnetic, &imu_ptr->acceleration, &imu_ptr->angularRate);


            imu_output_ptr->quaternion = imu_ptr->quaternion;
            imu_output_ptr->magnetic = imu_ptr->magnetic;
            imu_output_ptr->acceleration = imu_ptr->acceleration;
            imu_output_ptr->angularRate = imu_ptr->angularRate;


            imu_output_ptr->valid_data = true;
            imu_mutex_->unlock(); 
        }

		// std::cout << "got imu" << std::endl;
    }
}