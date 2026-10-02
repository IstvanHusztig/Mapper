# Spec Delta

## ADDED Requirements

### Requirement: Application name and window title
The application SHALL be named dMapper. The executable SHALL be `dMapper` (`dMapper.exe` on Windows), and the GUI window title SHALL be exactly "dMapper", without a version number or folder name. The GUI's texts (the Info window and dialogs) SHALL NOT refer to other products' workflows, such as Mandeye or HDMapping's registration steps.

#### Scenario: Window title during a run
- **WHEN** the GUI has started and a recording folder has been picked and is being processed
- **THEN** the window title still reads exactly "dMapper"

#### Scenario: Info window
- **WHEN** the user opens Info
- **THEN** the text describes dMapper and where its results are saved, and does not mention Mandeye
