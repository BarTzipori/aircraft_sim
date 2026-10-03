#include "Simulator.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kGravityMps2 = 9.80665;
constexpr int kPitchSearchIterations = 34;

double degreesToRadians(double angle_deg) { return angle_deg * kPi / 180.0; }

double moveToward(double current, double target, double maximum_change) {
    return current + std::max(-maximum_change, std::min(maximum_change, target - current));
}

// Stores the profile solved for a segment before the per-tick simulation begins.
struct SegmentProfile {
    Segment instruction;
    double initial_speed_mps;
    double initial_bank_rad;
    double peak_pitch_rad;
    double pitch_rate_rad_s;
};

double speedAt(const SegmentProfile& profile, double elapsed_s) {
    return moveToward(profile.initial_speed_mps, profile.instruction.target_speed_mps,
                      profile.instruction.chosen_acceleration_mps2 * elapsed_s);
}

double bankAt(const SegmentProfile& profile, double elapsed_s) {
    return moveToward(profile.initial_bank_rad,
                      degreesToRadians(profile.instruction.target_bank_deg),
                      degreesToRadians(profile.instruction.chosen_roll_rate_deg_s) * elapsed_s);
}

double pitchAt(const SegmentProfile& profile, double elapsed_s) {
    const double duration_s = profile.instruction.end_time_s - profile.instruction.start_time_s;
    // Ramp up, hold if time remains, and ramp down to level flight at the endpoint.
    const double pitch_magnitude = std::min({std::abs(profile.peak_pitch_rad),
                                             profile.pitch_rate_rad_s * elapsed_s,
                                             profile.pitch_rate_rad_s * (duration_s - elapsed_s)});
    return std::copysign(std::max(0.0, pitch_magnitude), profile.peak_pitch_rad);
}

double pitchRateAt(const SegmentProfile& profile, double elapsed_s) {
    if (std::abs(profile.peak_pitch_rad) < 1e-13) return 0.0;
    const double duration_s = profile.instruction.end_time_s - profile.instruction.start_time_s;
    const double ramp_duration_s = std::abs(profile.peak_pitch_rad) / profile.pitch_rate_rad_s;
    if (elapsed_s < ramp_duration_s - 1e-10) {
        return std::copysign(profile.pitch_rate_rad_s, profile.peak_pitch_rad);
    }
    if (elapsed_s >= duration_s - ramp_duration_s - 1e-10 && elapsed_s < duration_s - 1e-10) {
        return -std::copysign(profile.pitch_rate_rad_s, profile.peak_pitch_rad);
    }
    return 0.0;
}

double predictedAltitudeChange(const SegmentProfile& profile, double candidate_peak_pitch_rad,
                               double tick_frequency_hz) {
    SegmentProfile candidate = profile;
    candidate.peak_pitch_rad = candidate_peak_pitch_rad;
    const double duration_s = candidate.instruction.end_time_s - candidate.instruction.start_time_s;
    const int tick_count = static_cast<int>(std::llround(duration_s * tick_frequency_hz));
    double altitude_change_m = 0.0;
    // Use the same midpoint sampling as the vertical-position integrator.
    for (int tick_index = 0; tick_index < tick_count; ++tick_index) {
        const double midpoint_s = (tick_index + 0.5) / tick_frequency_hz;
        altitude_change_m += speedAt(candidate, midpoint_s) *
                             std::sin(pitchAt(candidate, midpoint_s)) / tick_frequency_hz;
    }
    return altitude_change_m;
}

std::vector<SegmentProfile> prepareProfiles(const Aircraft& aircraft, const FlightPlan& flight_plan,
                                            const State& initial_state, double tick_frequency_hz) {
    const AircraftLimits& limits = aircraft.limits();
    if (initial_state.speed_mps < limits.minimum_speed_mps ||
        initial_state.speed_mps > limits.maximum_speed_mps ||
        std::abs(initial_state.pitch_rad) > 1e-9 || std::abs(initial_state.bank_rad) > 1e-9 ||
        std::abs(-initial_state.down_m - flight_plan.segments().front().start_altitude_m) > 1e-7) {
        throw std::invalid_argument("Initial state must be level, wings level, at the requested altitude");
    }

    std::vector<SegmentProfile> profiles;
    double previous_speed_mps = initial_state.speed_mps;
    double previous_bank_rad = initial_state.bank_rad;
    for (const Segment& instruction : flight_plan.segments()) {
        const double duration_s = instruction.end_time_s - instruction.start_time_s;
        if (std::abs(std::llround(duration_s * tick_frequency_hz) / tick_frequency_hz - duration_s) > 1e-9) {
            throw std::invalid_argument("Segment boundaries must align with the tick interval");
        }
        const bool accelerating = instruction.target_speed_mps >= previous_speed_mps;
        const double allowed_acceleration = accelerating ? limits.maximum_acceleration_mps2
                                                         : limits.maximum_deceleration_mps2;
        const double requested_altitude_change_m = instruction.end_altitude_m - instruction.start_altitude_m;
        const bool climbing = requested_altitude_change_m >= 0;
        const double allowed_pitch_rate = climbing ? limits.maximum_climb_pitch_rate_deg_s
                                                   : limits.maximum_descent_pitch_rate_deg_s;
        const double allowed_pitch_angle = climbing ? limits.maximum_climb_pitch_deg
                                                    : limits.maximum_descent_pitch_deg;
        if (instruction.target_speed_mps < limits.minimum_speed_mps ||
            instruction.target_speed_mps > limits.maximum_speed_mps ||
            std::abs(instruction.target_bank_deg) > limits.maximum_bank_deg ||
            instruction.chosen_roll_rate_deg_s > limits.maximum_roll_rate_deg_s ||
            instruction.chosen_acceleration_mps2 > allowed_acceleration ||
            instruction.chosen_pitch_rate_deg_s > allowed_pitch_rate) {
            throw std::invalid_argument("Segment instruction exceeds aircraft limits");
        }
        if (std::abs(instruction.target_speed_mps - previous_speed_mps) >
                instruction.chosen_acceleration_mps2 * duration_s + 1e-8 ||
            std::abs(degreesToRadians(instruction.target_bank_deg) - previous_bank_rad) >
                degreesToRadians(instruction.chosen_roll_rate_deg_s) * duration_s + 1e-8) {
            throw std::invalid_argument("Speed or bank target cannot be reached before segment ends");
        }

        SegmentProfile profile{instruction, previous_speed_mps, previous_bank_rad, 0.0,
                               degreesToRadians(instruction.chosen_pitch_rate_deg_s)};
        const double maximum_peak_pitch_rad = std::min(degreesToRadians(allowed_pitch_angle),
                                                        profile.pitch_rate_rad_s * duration_s / 2.0);
        const double attainable_altitude_m = std::abs(predictedAltitudeChange(
            profile, std::copysign(maximum_peak_pitch_rad, requested_altitude_change_m), tick_frequency_hz));
        if (std::abs(requested_altitude_change_m) > attainable_altitude_m + 0.0001) {
            throw std::invalid_argument("Altitude target cannot be reached within segment time");
        }

        // Vertical displacement increases monotonically with peak pitch in this model.
        // Binary search finds the peak that reaches the requested end altitude.
        double lower_peak_rad = 0.0;
        double upper_peak_rad = maximum_peak_pitch_rad;
        for (int search_iteration = 0; search_iteration < kPitchSearchIterations; ++search_iteration) {
            const double candidate_peak_rad = (lower_peak_rad + upper_peak_rad) / 2.0;
            const double candidate_change_m = std::abs(predictedAltitudeChange(
                profile, std::copysign(candidate_peak_rad, requested_altitude_change_m), tick_frequency_hz));
            if (candidate_change_m < std::abs(requested_altitude_change_m)) {
                lower_peak_rad = candidate_peak_rad;
            } else {
                upper_peak_rad = candidate_peak_rad;
            }
        }
        profile.peak_pitch_rad = std::copysign((lower_peak_rad + upper_peak_rad) / 2.0,
                                               requested_altitude_change_m);
        profiles.push_back(profile);
        previous_speed_mps = instruction.target_speed_mps;
        previous_bank_rad = degreesToRadians(instruction.target_bank_deg);
    }
    return profiles;
}

struct PositionRate {
    double north_mps;
    double east_mps;
    double down_mps;
    double yaw_rad_s;
};

PositionRate positionRateAt(const State& state, const SegmentProfile& profile, double elapsed_s) {
    const double speed_mps = speedAt(profile, elapsed_s);
    const double pitch_rad = pitchAt(profile, elapsed_s);
    const double bank_rad = bankAt(profile, elapsed_s);
    const double horizontal_speed_mps = speed_mps * std::cos(pitch_rad);
    // Simplified coordinated turn: bank changes yaw; pitch reduces horizontal speed.
    const double yaw_rate_rad_s = kGravityMps2 * std::tan(bank_rad) / horizontal_speed_mps;
    return {horizontal_speed_mps * std::cos(state.yaw_rad),
            horizontal_speed_mps * std::sin(state.yaw_rad),
            -speed_mps * std::sin(pitch_rad), yaw_rate_rad_s};
}

State advancePosition(State state, const PositionRate& rate, double elapsed_s) {
    state.north_m += rate.north_mps * elapsed_s;
    state.east_m += rate.east_mps * elapsed_s;
    state.down_m += rate.down_mps * elapsed_s;
    state.yaw_rad += rate.yaw_rad_s * elapsed_s;
    return state;
}

Sample makeSample(double time_s, State state, const SegmentProfile& profile, double elapsed_s) {
    state.speed_mps = speedAt(profile, elapsed_s);
    state.bank_rad = bankAt(profile, elapsed_s);
    state.pitch_rad = pitchAt(profile, elapsed_s);
    const double horizontal_speed_mps = state.speed_mps * std::cos(state.pitch_rad);
    const double yaw_rate_rad_s = kGravityMps2 * std::tan(state.bank_rad) / horizontal_speed_mps;
    const double along_path_acceleration_mps2 =
        std::abs(state.speed_mps - profile.instruction.target_speed_mps) < 1e-9 ? 0.0 :
        std::copysign(profile.instruction.chosen_acceleration_mps2,
                      profile.instruction.target_speed_mps - profile.initial_speed_mps);
    const double pitch_rate_rad_s = pitchRateAt(profile, elapsed_s);
    // Differentiate the NED velocity analytically to obtain NED acceleration.
    const double horizontal_acceleration_mps2 =
        along_path_acceleration_mps2 * std::cos(state.pitch_rad) -
        state.speed_mps * std::sin(state.pitch_rad) * pitch_rate_rad_s;
    return {time_s, state,
            horizontal_speed_mps * std::cos(state.yaw_rad),
            horizontal_speed_mps * std::sin(state.yaw_rad),
            -state.speed_mps * std::sin(state.pitch_rad),
            horizontal_acceleration_mps2 * std::cos(state.yaw_rad) -
                horizontal_speed_mps * std::sin(state.yaw_rad) * yaw_rate_rad_s,
            horizontal_acceleration_mps2 * std::sin(state.yaw_rad) +
                horizontal_speed_mps * std::cos(state.yaw_rad) * yaw_rate_rad_s,
            -along_path_acceleration_mps2 * std::sin(state.pitch_rad) -
                state.speed_mps * std::cos(state.pitch_rad) * pitch_rate_rad_s};
}
} // namespace

Simulator::Simulator(const Aircraft& aircraft, const FlightPlan& flight_plan,
                     State initial_state, double tick_frequency_hz)
    : aircraft_(aircraft), flight_plan_(flight_plan), initial_state_(initial_state),
      tick_frequency_hz_(tick_frequency_hz) {
    if (tick_frequency_hz_ <= 0) throw std::invalid_argument("Invalid tick frequency");
}

void Simulator::run(const std::function<void(const Sample&)>& emit_sample) const {
    const std::vector<SegmentProfile> profiles =
        prepareProfiles(aircraft_, flight_plan_, initial_state_, tick_frequency_hz_);
    State current_state = initial_state_;
    const double tick_duration_s = 1.0 / tick_frequency_hz_;
    for (const SegmentProfile& profile : profiles) {
        const double duration_s = profile.instruction.end_time_s - profile.instruction.start_time_s;
        const int tick_count = static_cast<int>(std::llround(duration_s * tick_frequency_hz_));
        for (int tick_index = 0; tick_index < tick_count; ++tick_index) {
            const double elapsed_s = tick_index * tick_duration_s;
            emit_sample(makeSample(profile.instruction.start_time_s + elapsed_s,
                                   current_state, profile, elapsed_s));
            // Midpoint integration evaluates the controls halfway through the tick.
            const PositionRate initial_rate = positionRateAt(current_state, profile, elapsed_s);
            const State midpoint_state = advancePosition(current_state, initial_rate, tick_duration_s / 2.0);
            const PositionRate midpoint_rate =
                positionRateAt(midpoint_state, profile, elapsed_s + tick_duration_s / 2.0);
            current_state = advancePosition(current_state, midpoint_rate, tick_duration_s);
        }
        if (std::abs(-current_state.down_m - profile.instruction.end_altitude_m) > 0.002) {
            throw std::runtime_error("Altitude integration error exceeds 2 mm");
        }
    }
    // Include the endpoint once, with settled pitch-rate and acceleration values.
    const SegmentProfile& final_profile = profiles.back();
    const double final_duration_s = final_profile.instruction.end_time_s - final_profile.instruction.start_time_s;
    emit_sample(makeSample(final_profile.instruction.end_time_s, current_state,
                           final_profile, final_duration_s));
}
