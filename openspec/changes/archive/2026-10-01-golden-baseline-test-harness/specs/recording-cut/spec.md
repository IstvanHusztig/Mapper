# Spec Delta

## Purpose

Cuts a time range out of a Mandeye-style recording (LAZ point clouds plus IMU CSV) into a new
input folder without altering the kept data, so short reference datasets can be made for fast
regression checks.

## ADDED Requirements

### Requirement: Cut a recording by time
The cut tool SHALL take an input folder, an output folder, and a start offset and duration in
seconds measured from the first IMU sample. It SHALL write to the output folder every LAZ point and
every IMU line whose timestamp falls in that range, keeping the input file names.

#### Scenario: First 30 seconds of 44
- **WHEN** the tool is run on `raw data/44` with start 0 s and duration 30 s
- **THEN** the output folder holds one LAZ and one IMU CSV, with the same names as the input, covering only the first 30 s of the recording

### Requirement: Kept data is unchanged
Kept LAZ points SHALL keep their exact stored coordinates, intensity, GPS time and all other point
attributes. The tool achieves this by reusing the input's point format, scale and offset. Kept
IMU lines SHALL be copied byte for byte, together with the header line. The input folder SHALL
never be modified.

#### Scenario: Point attributes preserved
- **WHEN** a point inside the range is read from the input and from the cut output
- **THEN** all its stored attributes are identical

#### Scenario: IMU lines preserved
- **WHEN** the cut IMU CSV is compared with the matching lines of the input CSV
- **THEN** the header and every kept line are byte-identical

### Requirement: Cut output processes like a recording
The pipeline SHALL accept the cut folder as a normal input folder, and the cut tool SHALL refuse
an output folder that already contains `.laz`, `.las` or `.csv` files.

#### Scenario: Process the short cut
- **WHEN** the Release CLI is run on the cut folder with `settings/lidar-vertical.toml`
- **THEN** it completes and writes a normal output folder

#### Scenario: Non-empty output folder
- **WHEN** the output folder already contains a `.csv` file
- **THEN** the tool exits non-zero without writing anything
