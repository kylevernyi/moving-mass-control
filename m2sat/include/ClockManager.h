#pragma once
#include <chrono>
#include <unordered_map>
#include <string>
#include <functional>

using namespace std::chrono;

class ClockManager {
public:
    void AddClock(const std::string& id, double interval) {
        clocks[id] = {interval, high_resolution_clock::now()};
    }

    std::pair<bool, duration<double>> Elapsed(const std::string& id) {
        auto now = high_resolution_clock::now();
        auto& clock = clocks[id];
        auto dt = now - clock.last_time;
        
        bool elapsed = (dt.count() >= clock.interval); 
        if (elapsed) {
            clock.last_time = now;
        }
        return {elapsed, dt};
        
    }

private:
    struct Clock {
        double interval;
        time_point<high_resolution_clock> last_time;
    };

    std::unordered_map<std::string, Clock> clocks;
};