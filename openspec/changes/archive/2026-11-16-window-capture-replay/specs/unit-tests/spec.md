## ADDED Requirements

### Requirement: Window capture tested on synthetic data
The unit tests SHALL run the window loop on the synthetic loop recording with a capture, without recorded data, and SHALL
check that the capture leaves the run unchanged, that the capture file gives the capture back, and that a replay with the
same settings repeats every captured window bit for bit.

#### Scenario: Capture, file and replay
- **WHEN** one lap of the loop recording is run with a capture of 5–12 s, the capture is written to a file, read back and replayed with the same settings
- **THEN** the poses of the run equal those of a run without capture, and every replayed window equals the captured one

#### Scenario: The range of a capture
- **WHEN** windows start at 0, 2, 4, 6 and 8 s and the capture range is 1–5 s
- **THEN** the captured windows are those starting at 2 and 4 s, and a range in which no window starts captures nothing
