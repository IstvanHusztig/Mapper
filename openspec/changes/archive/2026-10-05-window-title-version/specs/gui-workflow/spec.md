## MODIFIED Requirements

### Requirement: Application name and window title
The application SHALL be named dMapper. The executable SHALL be `dMapper` (`dMapper.exe` on Windows), and the GUI window title SHALL be exactly "dMapper v 0.1": the name and dMapper's version, without a folder name. The GUI's texts (dialogs, tooltips) SHALL NOT refer to other products' workflows, such as Mandeye or HDMapping's registration steps.

#### Scenario: Window title during a run
- **WHEN** the GUI has started and a recording folder has been picked and is being processed
- **THEN** the window title still reads exactly "dMapper v 0.1"

#### Scenario: Folder dialog
- **WHEN** the user chooses Start → Load data and process
- **THEN** the folder dialog asks for a recording folder (lidar*.laz + imu*.csv) and does not mention Mandeye
