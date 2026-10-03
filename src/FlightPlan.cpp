#include "FlightPlan.hpp"
#include <cmath>
#include <stdexcept>
#include <utility>

FlightPlan::FlightPlan(std::vector<Segment> segments) : segments_(std::move(segments)) {
    if (segments_.empty() || std::abs(segments_.front().start_time_s) > 1e-9) {
        throw std::invalid_argument("Flight plan must begin at time zero");
    }
    for (std::size_t segment_index = 0; segment_index < segments_.size(); ++segment_index) {
        const Segment& segment = segments_[segment_index];
        if (segment.end_time_s <= segment.start_time_s || segment.chosen_roll_rate_deg_s <= 0 ||
            segment.chosen_pitch_rate_deg_s <= 0 || segment.chosen_acceleration_mps2 <= 0) {
            throw std::invalid_argument("Invalid segment duration or chosen rate");
        }
        if (segment_index == 0) continue;
        const Segment& previous_segment = segments_[segment_index - 1];
        if (std::abs(segment.start_time_s - previous_segment.end_time_s) > 1e-9 ||
            std::abs(segment.start_altitude_m - previous_segment.end_altitude_m) > 1e-7) {
            throw std::invalid_argument("Time or altitude discontinuity between segments");
        }
    }
}
