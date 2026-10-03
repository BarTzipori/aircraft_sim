#pragma once

// All angle limits are positive magnitudes. Rates are measured in degrees/s.
struct AircraftLimits {
    double minimum_speed_mps;
    double maximum_speed_mps;
    double maximum_acceleration_mps2;
    double maximum_deceleration_mps2;
    double maximum_bank_deg;
    double maximum_roll_rate_deg_s;
    double maximum_climb_pitch_deg;
    double maximum_climb_pitch_rate_deg_s;
    double maximum_descent_pitch_deg;
    double maximum_descent_pitch_rate_deg_s;
};

class Aircraft {
public:
    explicit Aircraft(AircraftLimits limits);
    const AircraftLimits& limits() const { return limits_; }

private:
    AircraftLimits limits_;
};
