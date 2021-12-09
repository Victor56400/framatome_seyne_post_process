#ifndef SRC_DYNAMICPARAMETERS_HPP
#define SRC_DYNAMICPARAMETERS_HPP

#include <string>
#include <dynamic_reconfigure/Config.h>
#include <boost/any.hpp>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/emit.h>
#include <boost/date_time/posix_time/time_formatters_limited.hpp>
#include <utility>
#include <ros/ros.h>
#include <fstream>


namespace forssea_utilities {

/**
   * Function adding a parameter to a dyn reconf message
   * @param name name of the parameter to update
   * @param value new value for the parameter
   */
void addToMsg(const std::string &name, int value, dynamic_reconfigure::Config &conf);

/**
 * Function adding a parameter to a dyn reconf message
 * @param name name of the parameter to update
 * @param value new value for the parameter
 */
void addToMsg(const std::string &name, double value, dynamic_reconfigure::Config &conf);

/**
 * Function adding a parameter to a dyn reconf message
 * @param name name of the parameter to update
 * @param value new value for the parameter
 */
void addToMsg(const std::string &name, bool value, dynamic_reconfigure::Config &conf);

/**
 * Function adding a parameter to a dyn reconf message
 * @param name name of the parameter to update
 * @param value new value for the parameter
 */
void addToMsg(const std::string &name, const std::string &value, dynamic_reconfigure::Config &conf);

template<class T>
class InternalConfiguration {
public:
  T config_; /**< Internal configuration*/

  explicit InternalConfiguration(std::string ns) : ns(std::move(ns)) {
    init();
  }

  /**
   * Set a parameter from a config based on its level or its name
   * @tparam C parameter type
   * @tparam D type corresponding either to uint32_t (get by level) or std::string (get by name)
   * @param config current config
   * @param id level/name of the parameter
   * @param value new value to set
   */
  template<class C, class D>
  void set(const D &id, const C &value) {
    int &&index = find(id);
    dynamic_reconfigure::Config conf_msg;
    if (index != -1) {
      addToMsg(names_[index], value, conf_msg);
      unsafe_set(conf_msg);
    } else error(id, false);

  }

  /**
   * Set a vector of parameters based on their levels or their names
   * @tparam C parameter type
   * @tparam D type corresponding either to uint32_t (get by level) or std::string (get by name)
   * @param config current config
   * @param id level/name of the parameter
   * @param value new value to set
   */
  template<class C, class D>
  void set(const std::vector<D> &ids, const std::vector<C> &values) {
    assert(ids.size() == values.size());
    dynamic_reconfigure::Config conf_msg;
    for (uint i = 0; i < ids.size(); ++i) {
      int &&index = find(ids[i]);
      if (index != -1) {
        addToMsg(names_[index], values[i], conf_msg);
      } else error(ids[i], false);
    }
    unsafe_set(conf_msg);
  }

  /**
   * Get a parameter from a config based on its level or its name
   * @tparam C parameter type
   * @tparam D type corresponding either to uint32_t (get by level) or std::string (get by name)
   * @param config new config
   * @param id level/name of the parameter
   * @param value value to store the parameter
   */
  template<class C, class D>
  void get(const D &id, C &value) const {
    int &&index = find(id);
    if (index != -1) {
      value = unsafe_get<C>(index);
    } else {
      error(id, true);
      value = C();
    }
  }

  /**
   * Get parameter name from level
   * @param level level of the parameter
   * @return name of the corresponding parameter
   */
  std::string getName(uint32_t level) const {
    int &&index = find(level);
    if (index != -1) {
      return names_[index];
    } else {
      error(level, true);
      return "";
    }
  }

  std::string getName(const std::string &name) const {
    return name;
  }

  /**
   * Callback function for the service to save parameters
   * @param req request containing the folder in which to save files
   * @param res response containing the success variable
   * @return true
   */
  bool saveToFile(std::string path_in = "") const {
    if (getPath(path_in, path_in, true)) {
      YAML::Node config;
      for (uint i = 0; i < nb_params_; ++i) {
        YAML::Node node;
        node["type"] = types_[i];
        switch (t_name.at(types_[i])) {
          case 0:node["value"] = unsafe_get<std::string>(i);
            break;
          case 1:node["value"] = unsafe_get<int>(i);
            break;
          case 2:node["value"] = unsafe_get<bool>(i);
            break;
          case 3:node["value"] = unsafe_get<double>(i);
            break;
        }
        config[names_[i]] = node;
      }
      using boost::filesystem::path;
      using boost::posix_time::ptime;
      using boost::posix_time::second_clock;
      using boost::posix_time::to_simple_string;
      using boost::gregorian::day_clock;

      ptime todayUtc(day_clock::universal_day(), second_clock::universal_time().time_of_day());

      path dir = path(path_in);
      path file = dir / path(boost::posix_time::to_iso_extended_string(todayUtc));
      std::ofstream f(file.string());
      f << config;
      boost::filesystem::remove(dir / path("last"));
      boost::filesystem::create_symlink(file, dir / path("last"));
      return true;
    } else return false;
  }

  /**
   * Callback function for the service to load parameters
   * @param path_in path of the file to load parameters from
   * @return success
   */
  bool loadFromFile(std::string path_in = "") {
    if (getPath(path_in, path_in, false)) {
      YAML::Node config = YAML::LoadFile(path_in);
      dynamic_reconfigure::Config conf_msg;
      for (const auto &name:names_) {
        if (config[name] and config[name]["value"] and config[name]["type"]) {
          switch (t_name.at(config[name]["type"].template as<std::string>())) {
            case 0: addToMsg(name, config[name]["value"].template as<std::string>(), conf_msg);
              break;
            case 1: addToMsg(name, config[name]["value"].template as<int>(), conf_msg);
              break;
            case 2: addToMsg(name, config[name]["value"].template as<bool>(), conf_msg);
              break;
            case 3: addToMsg(name, config[name]["value"].template as<double>(), conf_msg);
              break;
          }
        } else {
          ROS_ERROR_STREAM("Invalid parameters file: missing parameter " << name);
          return false;
        }
      }
      unsafe_set(conf_msg);
      return true;
    } else return false;
  }

private:
  const std::vector<typename T::AbstractParamDescriptionConstPtr>
      &param_description_ = T::__getParamDescriptions__(); /**< Parameter description*/
  std::vector<uint32_t>
      levels_; /**< Vector storing the levels of the parameters in the order in which they are stored in a config*/
  std::vector<std::string>
      names_; /**< Vector storing the names of the parameters in the order in which they are stored in a config*/
  std::vector<std::string>
      types_; /**< Vector storing the types of the parameters in the order in which they are stored in a config*/
  int nb_params_ = 0; /**< Number of parameters in the configuration*/
  const std::map<std::string, int>
      t_name = {{"str", 0}, {"int", 1}, {"bool", 2}, {"double", 3}}; /**< Map between types and integers*/
  boost::filesystem::path default_path; /**< Default path for loading/saving files*/
  const std::string ns; /**< Namespace of the server/client, used as part of the default path*/

  /**
   * Loads the levels of all the dyn params of the config and stores them in a vector. Order is meaningful.
   */
  void init() {
    using boost::filesystem::path;
    char *fpf = getenv("FORSSEA_PARAMETERS_FOLDER");
    if (fpf) {
      default_path = path(fpf);
    } else {
      default_path = path(getenv("HOME")) / path(".forssea/parameters");
    }
    nb_params_ = param_description_.size();
    for (const auto &param : param_description_) {
      levels_.push_back(param->level);
      names_.push_back(param->name);
      types_.push_back(param->type);
    }
  }

  void error(uint32_t level, bool get) const {
    ROS_ERROR_STREAM("Could not " << (get ? "get" : "set") << " parameter with level " << int(level) << ".");
  }

  void error(const std::string &name, bool get) const {
    ROS_ERROR_STREAM("Could not " << (get ? "get" : "set") << " parameter with name " << name << ".");
  }

  /**
 * Get the index of a value in a vector of values (uint32_t)
 * @param val value
 * @param lst vector of value
 * @return index if found, -1 otherwise
 */
  int find(const uint32_t &val) const {
    auto it = std::find(levels_.begin(), levels_.end(), val);
    if (it != levels_.end()) {
      return std::distance(levels_.begin(), it);
    } else return -1;
  }

  /**
  * Get the index of a value in a vector of values (std::string)
  * @param val value
  * @param lst vector of value
  * @return index if found, -1 otherwise
  */
  int find(const std::string &val) const {
    auto it = std::find(names_.begin(), names_.end(), val);
    if (it != names_.end()) {
      return std::distance(names_.begin(), it);
    } else return -1;
  }

/**
 * Function to get a parameter from the internal config. Unsafe because the index is not checked.
 * @tparam C type of the parameter
 * @param index index of the parameter in the config object
 * @return parameter
 */
  template<class C>
  C unsafe_get(int index) const {
    boost::any val;
    param_description_.at(index)->getValue(config_, val);
    return boost::any_cast<C>(val);
  }

  /**
   * Update the server config and the internal config from a config message. Unsafe because does not check the config message.
   * @param msg config message
   */
  void unsafe_set(dynamic_reconfigure::Config &msg) {
    config_.__fromMessage__(msg);
    config_.__clamp__();
  }

  bool getPath(const std::string &path_in, std::string &path_out, bool dir = true) const {
    using boost::filesystem::path;
    using boost::filesystem::exists;
    using boost::filesystem::is_directory;
    using boost::filesystem::create_directories;
    path working_path;
    if (path_in.empty()) {
      working_path = default_path / path(ns);
    } else {
      working_path = path_in;
    }
    if (dir) {
      if (not exists(working_path)) {
        try {
          create_directories(working_path);
        } catch (std::system_error &) {
          ROS_WARN_STREAM("The provided path " << working_path << " does not exist and could not be created.");
          return false;
        }
      }
      if (not is_directory(working_path)) {
        ROS_WARN_STREAM("Expected directory path, got " << working_path);
        return false;
      }
    } else {
      if (not exists(working_path)) {
        ROS_WARN_STREAM("The provided path " << working_path << " does not exist.");
        return false;
      }
      if (is_directory(working_path)) {
        working_path /= path("last");
        if (not exists(working_path)) {
          ROS_WARN_STREAM("Expected file, got " << working_path << " and could not find " << working_path);
          return false;
        }
      }
    }
    path_out = working_path.string();
    return true;
  }

};

}

#endif //SRC_DYNAMICPARAMETERS_HPP
