#pragma once

// the command-line front end: dMapper <input_folder> <params.toml> <output_folder> [--from-snapshot <file>]
// --from-snapshot: the odometry is not run; its windows and run state come from an odometry snapshot of an earlier run with
// the same recording and odometry settings (save_odometry_snapshot), then loop closure, consistency and saving run as usual
int run_cli(int argc, char *argv[]);
