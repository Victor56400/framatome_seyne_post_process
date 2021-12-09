# Package: forssea_utilities

## Introduction
This package contains general helping functions for both c++ and python nodes, as well as general purpose nodes.

---

## Helpers
### Python
- `get_param(param_name, default_value)`: Search for the closest param name matching : private -> relative -> and goes 
up in the namespace hierarchy and returns the parameter's value if found or the default value.
- `Environment`: Environment parameter manager. Should be used to access environment variables.
- `DDynamicReconfigure` **DEPRECATED**: Dynamic reconfigure client/server wrapper for easier use and exception safety
- `DynamicParametersServer`: Dynamic reconfigure server wrapper for easier use and exception safety
- `DynamicParametersClient`: Dynamic reconfigure client wrapper for easier use and exception safety
- `Rate`: Reconfigurable ros Rate wrapper
- `to_angle`: clip a value between `-pi` and `pi`
- `Grafcet` : Grafcet design pattern for implementing low-level device sequential behaviors

### C++
- `Matrix`: Class for low-level matrix manipulation, used for thrust allocation algorithms.
- `Environment`: Environment parameter manager. Should be used to access environment variables.
- `DynamicParametersServer`: Dynamic reconfigure server wrapper for easier use and exception safety
- `DynamicParametersClient`: Dynamic reconfigure client wrapper for easier use and exception safety
- `Rate`: Reconfigurable ros Rate wrapper
- `getParam`: Looks for a parameter in the parameter server
- `operator<<`: Display operator for std::vectors and boost::container::vector
- `operator+=`: Addition of an std::vector and a scalar
- `operator%=`: Modulo operation of an std::vector by a scalar
- `sgn`: sign function
- `inRange`: check that a value is in an interval
- `angle`: clip a value between `-pi` and `pi`

## Saving dynamic parameters
When using forssea_utilities dyn param helpers (`DynamicParametersServer` and `DynamicParametersClient`), you have the
opportunity to save/load/reset parameters in configuration files by calling a service and publishing on a topic:
- `/save_all` (topic, `std_msgs/Empty`) : Publishing on this topic will make every dyn param server save its parameters
- `save_parameters` (service, `forssea_msgs/SetString`) : Calling this service will trigger the backup of the parameters.
The string is optional, and indicates where to save the parameters (*enter a directory path*).
- `load_parameters` (service, `forssea_msgs/SetString`) : Calling this service will trigger the loading of the parameters.
The string is optional, and indicates from where to load the parameters (*enter a file path*).
- `reset_parameters` (service) : Calling this service will reset all the parameters to their default value (set in the .cfg file).

By default, the parameters are saved and loaded from the directory `~/.forssea/parameters/...`, in subdirectories named
after the server's namespace.
The default directory can be changed by setting the environment variable `FORSSEA_PARAMETERS_FOLDER` (e.g.
`export FORSSEA_PARAMETERS_FOLDER="$HOME/Documents/parameters"`).

## Generating a grafcet : `create_grafcet.py`
The script is available to generate all the classes required to implement a specific grafcet.
The grafcet structure must first be described in a yaml file (see `grafcet_description.yaml` in the example dir).
The script can then be called with the following possible options:
- `-c` : C++ header and source file are generated, instead of the python default version
- `-i <inputfile(yaml filename,w/o extension)>` : indicate the description file name if different from local 
`grafcet_description` (without the `.yaml` extension)
- `-o <outputfile(py/h&cpp filename,w/o extension)>` : indicate the output file radix name (without the extension,
which will be .py / .hpp & .cpp depending on the -c option) 
Once the files are generated, paste them in you repo and implement the step actions and the transition conditions!
