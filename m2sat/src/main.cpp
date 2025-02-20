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

#define LOG_RATE_HZ (10)
#define MAIN_LOOP_RATE_HZ (50)

uint64_t getTimestamp();

/* IMU access variables */
std::mutex imu_mutex;
imu_data_vn_format_t imu_data;

/* ZMQ Communications */
zmq::context_t context(1);
zmq::socket_t socket(context, zmq::socket_type::pub);

bool kalman_filter_initialized = false;

int main()
{
    /* Add clocks for loop timing*/
    ClockManager clockManager;
    const auto loopDuration = duration<float>(1.0f / MAIN_LOOP_RATE_HZ);
    clockManager.AddClock("controller", 1.0f / CONTROLLER_RATE_HZ);
    clockManager.AddClock("kalman", 1.0f / KALMAN_RATE_HZ);
    clockManager.AddClock("log", 1.0f / LOG_RATE_HZ);
    clockManager.AddClock("telemetry", 1.0f / TELEMETRY_SEND_RATE_HZ);


    /* Start IMU communication*/
    ConnectAndConfigureIMU(&imu_data, &imu_mutex);
    

    std::mt19937 gen(std::chrono::steady_clock::now().time_since_epoch().count());  // Seed it!
    std::normal_distribution<> distrib_normal(0.0, 1.0); // Mean 0.0, standard deviation 1.0


    /* Groundstation communication */
    socket.bind("tcp://*:5555");  // Bind to TCP port 5555

    
    // telemetry_t recovered = fromProto(proto);     // Convert back to telemetry_t
    telemetry_t tele;

     while (true) {
        auto start = high_resolution_clock::now(); // timestamp

        /* update measurements by pulling most recent data from IMU */
        if (imu_mutex.try_lock()) {
            imu_data.PullMeasurement(&tele); // pulls measurement from IMU into our current telemetry
            imu_mutex.unlock();

            if (!kalman_filter_initialized)
            {
                InitKalmanFilter(tele.omega_b2i);
                kalman_filter_initialized = true;
            }
        } 

        tele.r_mass <<  distrib_normal(gen),  distrib_normal(gen),  distrib_normal(gen);
        tele.rdot_mass <<  distrib_normal(gen),  distrib_normal(gen),  distrib_normal(gen);
        tele.r_mass_commanded <<  distrib_normal(gen),  distrib_normal(gen),  distrib_normal(gen);
        tele.u_com <<  distrib_normal(gen),  distrib_normal(gen),  distrib_normal(gen);
        tele.u_actual <<  distrib_normal(gen),  distrib_normal(gen),  distrib_normal(gen);

        // Create and populate a telemetry message
        TelemetryMessage msg = toProto(tele); // Convert to Protobuf
        msg.set_time(getTimestamp());
    
        // Serialize to string
        std::string serialized_msg;
        msg.SerializeToString(&serialized_msg);

        // Send the message over ZeroMQ
        zmq::message_t zmq_msg(serialized_msg.data(), serialized_msg.size());
        socket.send(zmq_msg, zmq::send_flags::none);

        std::cout << "Sent telemetry: " << msg.time() << std::endl;

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