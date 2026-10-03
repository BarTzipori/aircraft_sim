#pragma once
#include "Aircraft.hpp"
#include "FlightPlan.hpp"
#include <functional>

// Position and orientation use North-East-Down (NED) coordinates.
// Pitch and bank are measured in radians internally; input/output use degrees.
struct State {
    double north_m;
    double east_m;
    double down_m;
    double speed_mps;
    double pitch_rad;
    double bank_rad;
    double yaw_rad;
};

struct Sample {
    double time_s;
    State state;
    double north_velocity_mps;
    double east_velocity_mps;
    double down_velocity_mps;
    double north_acceleration_mps2;
    double east_acceleration_mps2;
    double down_acceleration_mps2;
};

class Simulator {
public:
    Simulator(const Aircraft& aircraft, const FlightPlan& flight_plan,
              State initial_state, double tick_frequency_hz);
    void run(const std::function<void(const Sample&)>& emit_sample) const;

private:
    const Aircraft& aircraft_;
    const FlightPlan& flight_plan_;
    State initial_state_;
    double tick_frequency_hz_;
};
