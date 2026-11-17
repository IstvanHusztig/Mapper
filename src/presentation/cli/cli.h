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
