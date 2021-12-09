#ifndef SRC_RATE_HPP
#define SRC_RATE_HPP

#include <forssea_utilities/DynamicParametersServer.hpp>
#include <forssea_msgs/RateConfig.h>
#include <mutex>


namespace forssea_utilities {

/** Rate is a wrapper for ros Rate class that allows dynamically changing sleeping duration */
class Rate {
public:
  /**
   * Constructor
   * Initializes dyn. param. server.
   */
  Rate() : rate_(1), frequency_(1), server_() {
    server_.start(std::bind(&Rate::callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  /**
   * Make the program sleep for the specified duration
   */
  void sleep();

  /**
   * Getter for the current frequency of the internal rate object
   * @return frequency of the internal rate object
   */
  double frequency();

  typedef std::function<void(double)> CallbackType;

  /**
   * Callback setter
   * The provided callback will be called when a new frequency is received.
   * @param callback
   */
  void setCallback(const CallbackType &callback);

private:
  /**
   * Callback function for the frequency
   * @param config containing the new frequency
   * @param level level of the changed parameter
   */
  void callback(forssea_msgs::RateConfig &config, uint32_t level);

  ros::Rate rate_; /**< Internal rate object*/
  double frequency_; /**< Frequency of the internal rate object*/
  forssea_utilities::DynamicParametersServer<forssea_msgs::RateConfig>
      server_; /**< Dyn. param. server for the frequency*/
  std::mutex mutex_; /**< Mutex to avoid refreshing the rate during a sleep command */
  CallbackType callback_; /**< External callback called when the frequency changes */
};

}

#endif //SRC_RATE_HPP
