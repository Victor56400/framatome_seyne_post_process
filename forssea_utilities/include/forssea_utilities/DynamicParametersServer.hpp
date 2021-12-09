#ifndef SRC_DDYNAMIC_RECONFIGURE_SERVER_H
#define SRC_DDYNAMIC_RECONFIGURE_SERVER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <forssea_msgs/SetString.h>
#include <forssea_utilities/DynamicParameters.hpp>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>


namespace forssea_utilities {

/**
 * Wrapper for dynamic_reconfigure::Server, allowing easier handling of dyn params
 * @tparam T Config type
 */
template<class T>
class DynamicParametersServer {
public:
  /**
   * Constructor
   * @param nh see dynamic_reconfigure::Server
   */
  explicit DynamicParametersServer(ros::NodeHandle nh = ros::NodeHandle("~")) : internal_config_(nh.getNamespace()) {
    server_ = std::unique_ptr<dynamic_reconfigure::Server<T>>(new dynamic_reconfigure::Server<T>(mutex_, nh));
    save_srv_ = nh.advertiseService("save_parameters", &DynamicParametersServer<T>::saveCallback, this);
    load_srv_ = nh.advertiseService("load_parameters", &DynamicParametersServer<T>::loadCallback, this);
    reset_srv_ = nh.advertiseService("reset_parameters", &DynamicParametersServer<T>::resetCallback, this);
    save_sub_ = nh.subscribe<std_msgs::Empty>("/save_all", 1, &DynamicParametersServer<T>::saveAllCallback, this);
    if (internal_config_.loadFromFile()) updateServer();
  }

  /**
   * Starts the dyn reconf server
   * @param callback external callback function
   */
  void start(const typename dynamic_reconfigure::Server<T>::CallbackType &callback) {
    callback_ = callback;
    server_->setCallback(std::bind(&DynamicParametersServer<T>::configCallback,
                                   this,
                                   std::placeholders::_1,
                                   std::placeholders::_2));
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
    updateServer();
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
    updateServer();
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
    get(id, value);
    value = f(value);
  }

private:
  std::unique_ptr<dynamic_reconfigure::Server<T>> server_; /**< Dyn. reconf. server*/
  boost::recursive_mutex mutex_; /**< External mutex for dyn. reconf. server*/
  ros::ServiceServer save_srv_; /**< Save config to file service server*/
  ros::ServiceServer load_srv_; /**< Load config from file service server*/
  ros::ServiceServer reset_srv_; /**< Reset config from default values*/
  ros::Subscriber save_sub_; /**< Subscriber used to save or load default parameters*/
  typename dynamic_reconfigure::Server<T>::CallbackType callback_;
  InternalConfiguration<T> internal_config_;  /**< Internal configuration of the client*/

  /**
   * Callback function for the service to save parameters
   * @param req request containing the folder in which to save files
   * @param res response containing the success variable
   * @return true
   */
  bool saveCallback(forssea_msgs::SetStringRequest &req, forssea_msgs::SetStringResponse &res) {
    res.success = internal_config_.saveToFile(req.string);
    return true;
  }

  /**
   * Callback to save the parameters of the node when the save_all topic is triggered
   * @param msg incoming trigger
   */
  void saveAllCallback(std_msgs::Empty msg) {
    internal_config_.saveToFile();
  }

  /**
   * Callback function for the service to load parameters
   * @param req request containing the file to load parameters from
   * @param res response containing the success variable
   * @return true
   */
  bool loadCallback(forssea_msgs::SetStringRequest &req, forssea_msgs::SetStringResponse &res) {
    res.success = internal_config_.loadFromFile(req.string);
    callback_(internal_config_.config_, -1);
    updateServer();
    return true;
  }

  /**
   * Callback function for the service to reset parameters to defaults
   * @param req trigger request
   * @param res response containing the success variable
   * @return true
   */
  bool resetCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    server_->getConfigDefault(internal_config_.config_);
    callback_(internal_config_.config_, -1);
    updateServer();
    res.success = true;
    return true;
  }

  /**
   * Callback function for the parameter update
   * @param config new config
   * @param level level of the changed parameter
   */
  void configCallback(T &config, uint32_t level) {
    internal_config_.config_ = config;
    callback_(config, level);
  }

  /**
   * Update the server config and the internal config from a config message. Unsafe because does not check the config message.
   */
  void updateServer() {
    boost::recursive_mutex::scoped_lock lock(mutex_);
    server_->updateConfig(internal_config_.config_);
  }

};

}

#endif //SRC_DDYNAMIC_RECONFIGURE_SERVER_H
