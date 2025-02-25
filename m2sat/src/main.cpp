#include <iostream>
#include <zmq.hpp>
#include <thread>
#include <random>
#include "controller.h"
#include "telemetry_conversion.h"
#include "ClockManager.h"
#include "kalman.h"
#include "imu.h"
#include "tic.h"

#define MAIN_LOOP_RATE_HZ (200)
#define ACTUATION_RATE_HZ (100)

uint64_t getTimestamp();

/* IMU access variables */
std::mutex imu_mutex;
imu_data_vn_format_t imu_data;

/* ZMQ Communications */
zmq::context_t context(1);
zmq::socket_t telemetry_socket(context, zmq::socket_type::pub);

/* Stepper motors */
static const std::vector<uint8_t> tic_id = {0,1,2}; // x y z motor id in the firmware of the tic. must match!
int stepper_i2c_fd;  // only 1 file descriptor for the i2c bus required 

bool kalman_filter_initialized = false;


int main()
{
    static const uint64_t start_main = getTimestamp(); 

    /* Add clocks for loop timing*/
    ClockManager clockManager;
    const auto loopDuration = duration<float>(1.0f / MAIN_LOOP_RATE_HZ);
    clockManager.AddClock("controller", 1.0f / CONTROLLER_RATE_HZ);
    clockManager.AddClock("kalman", 1.0f / KALMAN_RATE_HZ);
    clockManager.AddClock("telemetry", 1.0f / TELEMETRY_SEND_RATE_HZ);
    clockManager.AddClock("actuation", 1.0f / ACTUATION_RATE_HZ);


    /* Start IMU communication*/
    ConnectAndConfigureIMU(&imu_data, &imu_mutex);
    
    /* Start TIC communication */
    int stepper_i2c_fd = open_i2c_device(TIC_I2C_ADDRESS_DEVICE);
    if (stepper_i2c_fd < 0) 
    {
        // std::cout << "could not connect to i2c bus" << std::endl;
    } 
    

    /* Groundstation communication */
    telemetry_socket.bind("tcp://*:5555");  // Bind to TCP port 5555

    
    telemetry_t tele;
    telemetry_t controller_output;

    while (true) {
        auto start = high_resolution_clock::now(); // timestamp

        /* update measurements by pulling most recent data from IMU */
        if (imu_mutex.try_lock()) {
            imu_data.PullMeasurement(&tele); // pulls measurement from IMU into our current telemetry
            imu_mutex.unlock();

            if (!kalman_filter_initialized)
            {
                InitKalmanFilter(tele.omega_b2i_B, tele.q_b2i);
                kalman_filter_initialized = true;
            }
        } 

        /* Get Mass Positions and Velocities from motor controllers */
        int32_t motor_positions[3]; int32_t motor_velocities[3];
        for (int i = 0; i<3; i++)
        {
            tic_get_current_position(stepper_i2c_fd, tic_id.at(i), &motor_positions[i]); // puts data into motor_positions
            tic_get_current_velocity(stepper_i2c_fd, tic_id.at(i), &motor_velocities[i]); // puts data into motor_velocities
        }
        // Map shaft position and velocity to moving mass position and velocity
        tele.r_mass = ConvertMotorPositionToMassPosition(motor_positions[0], motor_positions[1], motor_positions[2]);
        tele.rdot_mass = ConvertMotorSpeedToMassVelocity(motor_velocities[0], motor_velocities[1], motor_velocities[2]);

        /* Run the controller */
        auto check_controller_clock = clockManager.Elapsed("controller");        
        if (check_controller_clock.first && (imu_data.valid_data == true)) // if sufficient time elapsed and we have imu data
        {
            controller_output = Controller(tele, check_controller_clock.second.count());
            controller_output.time = getTimestamp() - start_main;
        }

        /* Actuate motors */
        if (clockManager.Elapsed("actuation").first)
        {
            std::vector<int32_t> motor_shaft_position_pulses = ConvertMassPositionToMotorPosition(
                controller_output.r_mass_commanded.x(), controller_output.r_mass_commanded.y(), controller_output.r_mass_commanded.z());
            // Send motor position commands to motors
            for (uint8_t i = 0; i<3; i++) {
                tic_exit_safe_start(stepper_i2c_fd, tic_id.at(i));
                tic_set_target_position(stepper_i2c_fd,  tic_id.at(i), motor_shaft_position_pulses.at(i));
            }
        }

        /* Send telemetry to groundstation */
        if (clockManager.Elapsed("telemetry").first) 
        {
            // Create and populate a telemetry message
            TelemetryMessage msg = toProto(controller_output); // Convert to Protobuf        
            // Serialize to string
            std::string serialized_msg;
            msg.SerializeToString(&serialized_msg);
            zmq::message_t zmq_msg(serialized_msg.data(), serialized_msg.size());
            telemetry_socket.send(zmq_msg, zmq::send_flags::none);
            controller_output.Disp();
        }
        
        /* Loop timing business */
        auto end = high_resolution_clock::now();
        duration<float> workTime = end - start; // Calculate the elapsed time for the work
        duration<float> sleepTime = loopDuration - workTime;  // Calculate remaining time to wait to achieve the desired frequency
        // If the work took longer than the loop duration, we skip sleeping, else sleep
        if (sleepTime > duration<float>::zero()) {
            std::this_thread::sleep_for(sleepTime);
        }    
    }
}




uint64_t getTimestamp() {
    // Get the current time point from the system clock
    auto now = std::chrono::system_clock::now();
    // Convert the time point to a duration since epoch
    auto duration = now.time_since_epoch();
    // Convert the duration to milliseconds
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    return static_cast<uint64_t>(milliseconds);
}