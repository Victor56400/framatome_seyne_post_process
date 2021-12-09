# Changelog
_cf : https://keepachangelog.com/en/0.3.0/_

## [Unreleased]

## [3.9.0] - 31/03/20
### Minor
- Added SetString.srv (Auguste Bourgois)
- Added Rate.cfg (Auguste Bourgois)
- Removed PIDInput.msg (Auguste Bourgois)


## [3.6.0] - 10/12/19
### Minor
- Added SetInt service type
- added setDACVoltage service
- added sonar.msg
- added a new conversion helper tf2state
- added a header to the waypoint msg


## [3.1.1] - 06/08/19
### Major
- RobotState header.frame is the global frame & local_frame_id field was introduced instead

### Minor
- Added useful helpers functions and removed unnecessayr returns

### Patch
- Bug correction


## [2.18.0] - 24/04/19
### Minor
- Added HighGains config file
- Added VectorField message


## [2.16.0] - 16/04/19
### Minor
- Added fields in Path.action and Waypoints.srv to have more info
- Added DOFs weights in thrust allocation cfg file for better tuning


## [2.14.1] - 08/04/19
### Patch
- Added Environment.cfg to CMakeLists.txt


## [2.14.0] - 05/04/19
### Minor
- Added Environment.cfg


## [2.13.1] - 20/03/19
### Minor
- Added state2and & ang2state helpers in cpp

### Patch
- Corrected state2array function


## [2.12.0] - 18/03/19
### Minor
- Added angular acceleration field in RobotState message
- Added helpers functions for angular acceleration in python
- Added EngineOrder service type


## [2.9.1] - 12/03/19
### Minor
- Added state2array and array2state functions
- Deserialize as float in python helpers functions
- Corrected return type in doc
- Added path action and waypoints service, and wpt2spt function
- added message & service files for exploration stack
- added action files for exploration stack
- implemented state2array and array2state functions

### Patch
- corrected bug in pos2_ functions (cast values to floats)


## [2.2.0] - 06/02/2019
### Minor
- As no Unreleased paragraph was updated, we will say there is only one feature increment in this version.


