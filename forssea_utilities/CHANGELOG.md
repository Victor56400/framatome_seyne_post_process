# Changelog
_cf : https://keepachangelog.com/en/0.3.0/_

## [Unreleased]
### Minor
- CORE-2 Make the G7 pattern available in C++ language (David Barral)


## [5.9.0] - 20/05/20
### Minor
- added a usefule method to convert a geometry_msgs::Quaternion to euler  (Alaa El Jawad)


## [5.8.2] - 02/06/20
### Patch
- Corrected bug in deprecated python API (Auguste Bourgois)


## [5.8.1] - 02/06/20
### Patch
- Corrected a bug occuring when building code on ubuntu 16 (Auguste Bourgois)


## [5.8.0] - 15/04/20
### Minor
- python grafcet design pattern (grafcet.py)
- grafcet generation script (create_grafcet.py + )
- Grafcet design pattern (dbarral-forssea)
- Grafcet design pattern addition (David Barral)


## [5.4.4] - 15/04/20
### Patch
- Corrected bug in dyn param server service creation (Auguste Bourgois)


## [5.4.3] - 23/03/20
### Patch
- Corrected time bug in dyn param client (due to dyn param library) (Auguste Bourgois)


## [5.4.2] - 18/03/20
### Major
- Removed CARE, DARE and DRAKE libraries, moved them to controllers_feedback_linearization (Auguste Bourgois)

### Minor
- Added dynamic frequency feature (Auguste Bourgois)
- Added wrapper (cpp) for dynamic reconfigure (Auguste Bourgois)
- Added new wrapper (python) for dynamic reconfigure (Auguste Bourgois)
- Added dynamic parameter save/reset from file feature (Auguste Bourgois)

### Patch
- Reorganized code (Auguste Bourgois)
- Added documentation for dyn params helpers (Auguste Bourgois)


## [4.7.1] - 10/12/19
### Minor
- moved various scripts to forssea_tools (see commits)

### Patch
- added type hints


## [4.6.3] - 10/12/19
### Patch
- improved error message for easier debugging
- use gui arg to control joint states
- removed deprecated documentation


## [4.6.0] - 07/08/19
### Minor
- Added general upload.launch
- Updated dependencies


## [4.4.1] - 24/04/19
### Minor
- Removed unused functions

### Patch
- Reformatted files


## [4.3.0] - 16/04/19
### Minor
- Did some optimisation in cpp code
- Added diag function for matrix code
- Improved error message for python dynamic reconfigure


## [4.0.1] - 08/04/19
### Major
- Implemented dynamic reconfigure client for environment parameters, and simple node implementing a server

### Patch
- Corrected import bug in helpers.py


## [3.2.1] - 04/04/19
### Patch
- xbox_joysick_safety is now executable


## [3.2.0] - 28/03/19
### Minor
- Added inline keyword for accessors in matrix.hpp


## [3.1.1] - 18/03/19
### Patch
- corrected type in get_param in cpp


## [3.1.0] - 05/03/19
### Major
- Remove wrench_adapter from this package

### Minor
- Cleaned dynamic reconfigure helper for python


## [2.2.1] - 11/02/2019
### Minor
- Removed unused functions in helper library for dynamic reconfigure in python.
- Adjusted default namespace for dynamic reconfigure in python.


## [2.1.0] - 06/02/2019
### Minor
- Added wrench_adapter, for use with the thrusters_tester node and the Thrust allocation node


