# Aircraft trajectory simulator

C++14 kinematic simulator with timed flight instructions loaded from JSON. The JSON contains **only the route**; no segment has an end time. `main.cpp` still supplies the aircraft limits, initial NED state and tick frequency (2000 Hz). Speed is in m/s; NED position and altitude are in metres.

## Build and run

```bash
g++ -std=c++14 -O2 -Wall -Wextra -Wpedantic -I include src/*.cpp -o aircraft_sim
./aircraft_sim flight_plan.json
```

The first command runs without writing a trajectory file. To export CSV for the separate visualizer, give an optional second argument:

```bash
./aircraft_sim flight_plan.json trajectory.csv
python -m pip install numpy matplotlib pillow
python render_aircraft_route.py trajectory.csv --output-prefix aircraft_route
```

For a long simulation, stream the CSV to a compressed file:

```bash
./aircraft_sim flight_plan.json - | gzip > trajectory.csv.gz
python render_aircraft_route.py trajectory.csv.gz --output-prefix aircraft_route
```

The renderer creates `aircraft_route.png` and `aircraft_route.gif`. Each route segment specifies the eight fields shown in `flight_plan.json`. A segment ends when the next one begins. The host supplies the final simulation end time (260 s in the example `main.cpp`). Unknown, duplicate, missing or invalid fields are rejected. The route loader uses a strict JSON subset appropriate for this schema: a top-level object with a `segments` array of objects containing numeric values. The fields are ASCII strings; other JSON value types and Unicode escapes are not supported. Segment times and altitudes must be continuous, and durations must be whole ticks.

## Project layout

- `include/Aircraft.hpp`, `src/Aircraft.cpp`: aircraft limits and validation.
- `include/FlightPlan.hpp`, `src/FlightPlan.cpp`: timed segment data and continuity checks.
- `include/FlightPlanJson.hpp`, `src/FlightPlanJson.cpp`: reads the route-only JSON once before simulation.
- `include/Simulator.hpp`, `src/Simulator.cpp`: control profiles, feasibility checks, per-tick integration and sample output.
- `src/main.cpp`: example aircraft, initial state and optional CSV export.
- `render_aircraft_route.py`: separate graph and animation generator.

The simulator solves for a ramp/hold/ramp pitch profile that reaches each segment's target altitude and returns to level flight at its endpoint. Bank may carry into the next segment. Its output samples contain time, NED position, NED velocity, NED acceleration, pitch, roll and yaw. Position and yaw are integrated per tick with the midpoint method. The model assumes pitch equals flight-path angle, a coordinated-turn approximation, no wind and no sideslip. It does not solve engine thrust, lift, drag or combined flight-envelope constraints.
