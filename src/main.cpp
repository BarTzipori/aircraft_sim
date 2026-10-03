#include "Simulator.hpp"
#include "FlightPlanJson.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

void writeSample(std::ostream& output_csv, const Sample& sample) {
    output_csv << sample.time_s << ','
               << sample.state.north_m << ',' << sample.state.east_m << ',' << sample.state.down_m << ','
               << sample.north_velocity_mps << ',' << sample.east_velocity_mps << ','
               << sample.down_velocity_mps << ',' << sample.north_acceleration_mps2 << ','
               << sample.east_acceleration_mps2 << ',' << sample.down_acceleration_mps2 << ','
               << sample.state.pitch_rad * kRadiansToDegrees << ','
               << sample.state.bank_rad * kRadiansToDegrees << ','
               << sample.state.yaw_rad * kRadiansToDegrees << '\n';
}
} // namespace

int main(int argument_count, char** arguments) {
    try {
        if (argument_count < 2 || argument_count > 3) {
            std::cerr << "Usage: aircraft_sim flight_plan.json [trajectory.csv | -]\n";
            return 1;
        }
        // 1. Construct the aircraft with its speed, acceleration and attitude limits.
        const Aircraft aircraft({50, 150, 3, 4, 30, 10, 5, 1, 5, 1});

        // 2. Read only the timed flight instructions from JSON.
        const double simulation_end_time_s = 260.0; // Supplied by the host, outside the route JSON.
        const FlightPlan flight_plan = loadFlightPlanJson(arguments[1], simulation_end_time_s);
        const State initial_state{0, 0, 0, 100, 0, 0, 0};

        // CSV is optional. The simulator itself returns samples without file output.
        const bool output_requested = argument_count == 3;
        std::ofstream output_file;
        if (output_requested && std::string(arguments[2]) != "-") {
            output_file.open(arguments[2]);
            if (!output_file) throw std::runtime_error("Cannot open CSV output");
        }
        std::ostream* output_csv = output_requested ?
            (output_file.is_open() ? static_cast<std::ostream*>(&output_file)
                                   : static_cast<std::ostream*>(&std::cout)) : nullptr;
        if (output_csv) {
            *output_csv << "time_s,north_m,east_m,down_m,v_n_mps,v_e_mps,v_d_mps,"
                           "a_n_mps2,a_e_mps2,a_d_mps2,pitch_deg,roll_deg,yaw_deg\n";
            *output_csv << std::fixed << std::setprecision(9);
        }

        // 3. Simulate at 2000 Hz. The host may use samples directly without CSV.
        Simulator simulator(aircraft, flight_plan, initial_state, 2000);
        Sample final_sample{};
        simulator.run([&](const Sample& sample) {
            final_sample = sample;
            if (output_csv) writeSample(*output_csv, sample);
        });
        std::cerr << "t=" << final_sample.time_s << " NED=("
                  << final_sample.state.north_m << ',' << final_sample.state.east_m << ','
                  << final_sample.state.down_m << ") yaw_deg="
                  << final_sample.state.yaw_rad * kRadiansToDegrees << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
