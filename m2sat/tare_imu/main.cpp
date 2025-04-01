#include <iostream>
#include "vn/sensors.h" // Include this header file to get access to VectorNav sensors.
#include "vn/thread.h" // We need this file for our sleep function.
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

#define VN_IMU_BAUD_RATE 115200
#define VN_IMU_SERIAL_PORT "/dev/ttyUSB0"

int main()
{
    VnSensor vs;  // this needs to be defined outside of a temporary function stack like the ConnectAndConfigure, since it needs to stay alive
    vs.connect(VN_IMU_SERIAL_PORT,   VN_IMU_BAUD_RATE); // connect to sensor

    FilterBasicControlRegister reg = vs.readFilterBasicControl();
    reg.magMode = MAGNETICMODE_2D;
    vs.writeFilterBasicControl(reg, 1);

    vs.tare(1);

    std::cout << "IMU Tared\n"; 
}