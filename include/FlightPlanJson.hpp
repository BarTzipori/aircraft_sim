#pragma once
#include "FlightPlan.hpp"
#include <string>

// Loads only route instructions. Aircraft limits, initial state and tick rate
// remain owned by the host simulator.
// The host determines when the final segment ends; the JSON contains only
// segment start times and manoeuvre targets.
FlightPlan loadFlightPlanJson(const std::string& path, double simulation_end_time_s);
