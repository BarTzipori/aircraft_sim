#pragma once
#include <vector>

// Instructions for one continuous time interval. Positive altitude means up.
struct Segment {
    double start_time_s;
    double end_time_s;
    double target_bank_deg;             // Positive turns right; zero flies straight.
    double chosen_roll_rate_deg_s;
    double target_speed_mps;
    double chosen_acceleration_mps2;    // Positive magnitude for acceleration or deceleration.
    double start_altitude_m;
    double end_altitude_m;
    double chosen_pitch_rate_deg_s;     // Positive magnitude for climb or descent.
};

class FlightPlan {
public:
    explicit FlightPlan(std::vector<Segment> segments);
    const std::vector<Segment>& segments() const { return segments_; }

private:
    std::vector<Segment> segments_;
};
