#pragma once
#include <chrono>
#include <unordered_map>
#include <string>
#include <functional>

using namespace std::chrono;

class ClockManager {
public:
    void AddClock(const std::string& id, double interval) {
        clocks[id] = Clock{interval, high_resolution_clock::now()};
    }

    std::pair<bool, duration<double>> Elapsed(const std::string& id) {
        auto it = clocks.find(id);
        if (it == clocks.end()) {
            throw std::runtime_error("Clock ID not found: " + id);
        }
    
        auto now = high_resolution_clock::now();
        auto& clock = it->second;
        auto dt = now - clock.last_time;

        bool elapsed = (dt.count()*1e-9 >= clock.interval); 
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