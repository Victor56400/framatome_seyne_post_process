#ifndef SRC_DDYNAMIC_RECONFIGURE_CLIENT_H
#define SRC_DDYNAMIC_RECONFIGURE_CLIENT_H

#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include <forssea_utilities/DynamicParameters.hpp>
#include <forssea_msgs/SetString.h>


namespace forssea_utilities {

/**
 * Wrapper for dynamic_reconfigure::Client, allowing easier handling of dyn params
 * @tparam T config type
 */
template<class T>
class DynamicParametersClient {
public:
  /**
   * Constructor
   * @param nh see dynamic_reconfigure::Client
   */
  explicit DynamicParametersClient(const ros::NodeHandle &nh = ros::NodeHandle("~"))
      : internal_config_(nh.getNamespace()) {
    client_ = std::unique_ptr<dynamic_reconfigure::Client<T>>(new dynamic_reconfigure::Client<T>(nh.getNamespace()));
  }

  /**
   * Starts the dyn reconf client
   * @param callback external callback function
   */
  void start(const std::function<void(const T &)> &callback) {
    callback_ = callback;
    client_->setConfigurationCallback(std::bind(&DynamicParametersClient<T>::configCallback,
                                                this,
                                                std::placeholders::_1));
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
    internal_config_.get(id, value);
    ROS_DEBUG_STREAM("Parameter " << getName(id) << " updated to " << value);
  }

  /**
   * Get parameter name from level
   * @param level level of the parameter
   * @return name of the corresponding parameter
   */
  template<class D>
  std::string getName(const D &level) const {
    return internal_config_.getName(level);
  }

  /**
   * Get a parameter from a config based on its level or its name, a function being applied to the parameter
   * @tparam C parameter type
   * @tparam D type corresponding either to uint32_t (get by level) or std::string (get by name)
   * @param config new config
   * @param id level/name of the parameter
   * @param value value to store the parameter
   * @param f function being applied to the parameter
   */
  template<class C, class D>
  void get(const D &id, C &value, const std::function<C(C)> f) const {
    internal_config_.get(id, value);
    value = f(value);
  }

  /**
   * Set a parameter based on its level or its name
   * @tparam C parameter type
   * @tparam D type corresponding either to uint32_t (get by level) or std::string (get by name)
   * @param config current config
   * @param id level/name of the parameter
   * @param value new value to set
   */
  template<class C, class D>
  void set(T &config, const D &id, const C &value) {
    internal_config_.set(id, value);
    if (config != internal_config_.config_) config = internal_config_.config_;
    if (setConfiguration(internal_config_.config_)) {
      ROS_DEBUG_STREAM("Successfully applied new configuration.");
    } else {
      ROS_WARN("Could not set new configuration to server.");
    }
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
  void set(T &config, const std::vector<D> &ids, const std::vector<C> &values) {
    assert(ids.size() == values.size());
    internal_config_.set(ids, values);
    if (config != internal_config_.config_) config = internal_config_.config_;
    if (setConfiguration(internal_config_.config_)) {
      ROS_DEBUG_STREAM("Successfully applied new configuration.");
    } else {
      ROS_WARN("Could not set new configuration to server.");
    }
  }

private:
  InternalConfiguration<T> internal_config_; /**< Internal configuration of the client*/
  std::unique_ptr<dynamic_reconfigure::Client<T>> client_;
  std::function<void(const T &)> callback_;

  void configCallback(const T &config) {
    callback_(config);
  }

};

}

#endif //SRC_DDYNAMIC_RECONFIGURE_CLIENT_H
