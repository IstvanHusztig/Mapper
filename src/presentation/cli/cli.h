#pragma once

// the command-line front end: dMapper <input_folder> <params.toml> <output_folder> [--from-snapshot <file>]
// --from-snapshot: the odometry is not run; its windows and run state come from an odometry snapshot of an earlier run with
// the same recording and odometry settings (save_odometry_snapshot), then loop closure, consistency and saving run as usual
int run_cli(int argc, char *argv[]);

// dMapper --replay <window_capture.bin> <params.toml>: runs the captured windows again (window_replay.h) and reports how far
// they come out from the captured run. Exit 0: with the capture's own settings every window is identical bit for bit, or
// the settings differ (then the differences are only reported); 1: the files can't be read, or the same settings gave
// other poses (the replay doesn't reproduce the run).
int run_replay(const char *capture_file, const char *params_file);

// dMapper --replay-loop-closure <loop_closure_capture.bin> <params.toml> <output_folder> [--reject-tries i-j[,i-j...]]: runs the loop closure after the
// registrations again from a capture (loop_closure_replay.h) and writes the trajectory files of a result
// (trajectory_lio_<n>.csv, session_poses.mrp, session_ini_poses.mri), the parameters, loop_closure_edges.json (edges kept
// and dropped) and loop_closure_log.txt to the output folder. Exit 0: done, and with the capture's own loop closure
// settings every node identical bit for bit; 1: the files can't be read, a setting is refused (named), the settings ask
// for registrations the capture doesn't hold (named; nothing written), or the own settings did not reproduce the run.
// --reject-tries: the named captured tries count as rejected registrations (loop_closure_replay.h); a named try the capture
// doesn't hold exits 1 naming it; the comparison with the run then decides nothing (the trajectory is written).
int run_loop_closure_replay(const char *capture_file, const char *params_file, const char *output_folder, const char *reject_tries = nullptr);
