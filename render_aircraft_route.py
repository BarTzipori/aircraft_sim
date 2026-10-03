#!/usr/bin/env python3
"""Render a simulator CSV as a 3D route PNG and an animated airplane GIF.

Install: python -m pip install numpy matplotlib pillow
Run:     python render_aircraft_route.py trajectory.csv --output-prefix my_route

The input is the CSV produced by the C++ aircraft simulator. Matplotlib renders
both files locally; no network or external 3D application is required.
"""
import argparse
import csv
import gzip
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as pyplot
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy

REQUIRED = ("time_s", "north_m", "east_m", "down_m", "pitch_deg", "roll_deg", "yaw_deg")


def load_samples(csv_path, sample_period_s):
    """Stream a large 2000 Hz CSV, keeping the first, last and display samples."""
    samples = []
    open_csv = gzip.open if csv_path.suffix == ".gz" else Path.open
    with open_csv(csv_path, mode="rt", newline="", encoding="utf-8") as csv_file:
        csv_reader = csv.DictReader(csv_file)
        if not csv_reader.fieldnames or any(column_name not in csv_reader.fieldnames for column_name in REQUIRED):
            raise ValueError("CSV needs columns: " + ", ".join(REQUIRED))
        next_sample_time_s = 0.0
        last_sample = None
        for csv_row in csv_reader:
            last_sample = [float(csv_row[column_name]) for column_name in REQUIRED]
            if last_sample[0] + 1e-9 >= next_sample_time_s:
                samples.append(last_sample)
                next_sample_time_s = last_sample[0] + sample_period_s
    if last_sample is None:
        raise ValueError("CSV contains no samples")
    if samples[-1][0] != last_sample[0]:
        samples.append(last_sample)
    return numpy.asarray(samples)


def draw_graph(samples, output_path):
    time_s, north_m, east_m, down_m, pitch_deg, roll_deg, yaw_deg = samples.T
    altitude = -down_m  # NED's Down axis is positive downward.
    figure = pyplot.figure(figsize=(13, 7), facecolor="#fafcff")
    trajectory_axes = figure.add_subplot(121, projection="3d")
    trajectory_axes.plot(east_m, north_m, altitude, color="#2869d1", linewidth=2)
    label_times_s = numpy.linspace(time_s[0], time_s[-1], 6)
    for label_time_s in label_times_s:
        sample_index = numpy.argmin(abs(time_s - label_time_s))
        trajectory_axes.scatter([east_m[sample_index]], [north_m[sample_index]], [altitude[sample_index]], c="#e14840", s=25)
        trajectory_axes.text(east_m[sample_index], north_m[sample_index], altitude[sample_index] + 60, f"{time_s[sample_index]:.0f}s", fontsize=8)
    trajectory_axes.set(xlabel="East (m)", ylabel="North (m)", zlabel="Altitude (m)", title="3D trajectory")
    trajectory_axes.view_init(23, -61)
    trajectory_axes.set_box_aspect((1, 1, .72))
    top_view_axes = figure.add_subplot(122)
    top_view_axes.plot(east_m, north_m, linewidth=2, color="#2869d1")
    top_view_axes.scatter([east_m[0], east_m[-1]], [north_m[0], north_m[-1]], c="#e14840", s=40)
    top_view_axes.set(xlabel="East (m)", ylabel="North (m)", title="Top view")
    top_view_axes.grid(alpha=.25)
    top_view_axes.set_aspect("equal", adjustable="box")
    figure.suptitle(f"Aircraft route | {time_s[-1]:.1f} s | final altitude {altitude[-1]:.0f} m", fontsize=13)
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    pyplot.close(figure)


def aircraft_geometry(north_m, east_m, down_m, pitch_deg, roll_deg, yaw_deg):
    """A simple airplane in body coordinates: forward, right wing, down."""
    aircraft_body_vertices = numpy.array([[110, 0, 0], [-80, 0, 0], [-50, -85, 0],
                     [-50, 85, 0], [-70, -32, -30], [-70, 32, -30], [0, 0, 0]])
    pitch_rad, roll_rad, yaw_rad = numpy.deg2rad([pitch_deg, roll_deg, yaw_deg])
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    cos_pitch, sin_pitch = math.cos(pitch_rad), math.sin(pitch_rad)
    cos_roll, sin_roll = math.cos(roll_rad), math.sin(roll_rad)
    # Rotate the body-frame forward/right/down axes into NED coordinates.
    forward_axis = numpy.array([cos_pitch * cos_yaw, cos_pitch * sin_yaw, -sin_pitch])
    level_right_axis = numpy.array([-sin_yaw, cos_yaw, 0.0])
    level_down_axis = numpy.cross(forward_axis, level_right_axis)
    body_right_axis = level_right_axis * cos_roll + level_down_axis * sin_roll
    body_down_axis = level_down_axis * cos_roll - level_right_axis * sin_roll
    vertices_ned = (aircraft_body_vertices[:, 0, None] * forward_axis
                    + aircraft_body_vertices[:, 1, None] * body_right_axis
                    + aircraft_body_vertices[:, 2, None] * body_down_axis)
    vertices_ned += numpy.array([north_m, east_m, down_m])
    return vertices_ned[:, 1], vertices_ned[:, 0], -vertices_ned[:, 2]


def draw_animation(samples, output_path, frames_per_second, max_frames):
    time_s, north_m, east_m, down_m, pitch_deg, roll_deg, yaw_deg = samples.T
    altitude = -down_m
    figure = pyplot.figure(figsize=(9, 7), facecolor="#fcfdff")
    trajectory_axes = figure.add_subplot(111, projection="3d")
    trajectory_axes.plot(east_m, north_m, altitude, color="#b7c9e6", linewidth=1.6, alpha=.9)
    def axis_bounds(values, minimum_span):
        minimum_value, maximum_value = float(numpy.min(values)), float(numpy.max(values))
        padding = max(minimum_span, (maximum_value - minimum_value) * .07)
        return minimum_value - padding, maximum_value + padding
    trajectory_axes.set(xlim=axis_bounds(east_m, 200), ylim=axis_bounds(north_m, 200), zlim=axis_bounds(altitude, 150),
           xlabel="East (m)", ylabel="North (m)", zlabel="Altitude (m)")
    trajectory_axes.view_init(25, -65)
    trajectory_axes.grid(alpha=.17)
    trail, = trajectory_axes.plot([], [], [], linewidth=2.7, color="#215cce")
    aircraft_edges = ((0, 1), (2, 6), (6, 3), (4, 1), (5, 1), (4, 5))
    aircraft_lines = [trajectory_axes.plot([], [], [], color="#e14a3d", linewidth=3)[0] for _ in aircraft_edges]
    position_marker = trajectory_axes.scatter([], [], [], s=30, color="#e14a3d")
    status_label = trajectory_axes.text2D(.03, .96, "", transform=trajectory_axes.transAxes, fontsize=12, color="#162b4a")
    frame_indices = numpy.unique(numpy.linspace(0, len(time_s) - 1, min(max_frames, len(time_s))).round().astype(int))
    def update(frame_number):
        sample_index = frame_indices[frame_number]
        trail.set_data(east_m[:sample_index + 1], north_m[:sample_index + 1])
        trail.set_3d_properties(altitude[:sample_index + 1])
        vertex_east, vertex_north, vertex_altitude = aircraft_geometry(north_m[sample_index], east_m[sample_index], down_m[sample_index], pitch_deg[sample_index], roll_deg[sample_index], yaw_deg[sample_index])
        for aircraft_line, (start_vertex, end_vertex) in zip(aircraft_lines, aircraft_edges):
            aircraft_line.set_data((vertex_east[start_vertex], vertex_east[end_vertex]), (vertex_north[start_vertex], vertex_north[end_vertex]))
            aircraft_line.set_3d_properties((vertex_altitude[start_vertex], vertex_altitude[end_vertex]))
        position_marker._offsets3d = ([east_m[sample_index]], [north_m[sample_index]], [altitude[sample_index]])
        status_label.set_text(f"t = {time_s[sample_index]:.0f} s   altitude = {altitude[sample_index]:.0f} m   bank = {roll_deg[sample_index]:.0f}°")
        return [trail, *aircraft_lines, position_marker, status_label]
    animation = FuncAnimation(figure, update, frames=len(frame_indices), interval=1000 / frames_per_second, blit=False)
    animation.save(output_path, writer=PillowWriter(fps=frames_per_second), dpi=95)
    pyplot.close(figure)


def main():
    argument_parser = argparse.ArgumentParser(description=__doc__)
    argument_parser.add_argument("csv", type=Path, help="Trajectory CSV produced by aircraft_sim")
    argument_parser.add_argument("--output-prefix", type=Path, default=Path("aircraft_route"))
    argument_parser.add_argument("--sample-period", type=float, default=.1, help="Seconds between retained plot samples")
    argument_parser.add_argument("--frames", type=int, default=66, help="Maximum number of GIF frames")
    argument_parser.add_argument("--fps", type=int, default=13)
    arguments = argument_parser.parse_args()
    if arguments.sample_period <= 0 or arguments.frames < 2 or arguments.fps <= 0:
        argument_parser.error("Sample period, FPS and frame count must be positive; frames >= 2")
    samples = load_samples(arguments.csv, arguments.sample_period)
    png_path = arguments.output_prefix.with_suffix(".png")
    gif_path = arguments.output_prefix.with_suffix(".gif")
    png_path.parent.mkdir(parents=True, exist_ok=True)
    draw_graph(samples, png_path)
    draw_animation(samples, gif_path, arguments.fps, arguments.frames)
    print(f"Saved {png_path} and {gif_path} ({len(samples)} trajectory points)")

if __name__ == "__main__":
    main()
