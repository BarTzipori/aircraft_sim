#include "Aircraft.hpp"
#include <stdexcept>

Aircraft::Aircraft(AircraftLimits limits) : limits_(limits) {
    if (limits.minimum_speed_mps <= 0 || limits.maximum_speed_mps < limits.minimum_speed_mps ||
        limits.maximum_acceleration_mps2 <= 0 || limits.maximum_deceleration_mps2 <= 0 ||
        limits.maximum_bank_deg <= 0 || limits.maximum_bank_deg >= 80 ||
        limits.maximum_roll_rate_deg_s <= 0 || limits.maximum_climb_pitch_deg <= 0 ||
        limits.maximum_climb_pitch_deg >= 80 || limits.maximum_descent_pitch_deg <= 0 ||
        limits.maximum_descent_pitch_deg >= 80 ||
        limits.maximum_climb_pitch_rate_deg_s <= 0 || limits.maximum_descent_pitch_rate_deg_s <= 0) {
        throw std::invalid_argument("Invalid aircraft limits");
    }
}
