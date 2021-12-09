#ifndef SRC_ENVIRONMENT_HPP
#define SRC_ENVIRONMENT_HPP

#include <forssea_msgs/EnvironmentConfig.h>
#include <forssea_utilities/DynamicParametersClient.hpp>
#include <eigen3/Eigen/Dense>


namespace forssea_utilities {

/**
 * Environment is a wrapper to get proper environment parameters from the ROS parameters server
 */
class Environment {
public:
  /**
   * Constructor
   * @param name namespace of the dynamic reconfigure client
   */
  explicit Environment(const ros::NodeHandle &nh = ros::NodeHandle("/environment"));

  /**
   * Returns local gravity value
   * @return local gravity value
   */
  inline const double &g() const { return g_; }

  /**
   * Returns local gravity vector
   * @return local gravity vector
   */
  inline Eigen::Vector3d gv() const { return {0, 0, g_}; }

  /**
   * Returns latitude coordinate of the origin of the environment of the robot
   * @return latitude coordinate of the origin of the environment of the robot
   */
  inline const double &lat0() const { return lat0_; }

  /**
   * Returns longitude coordinate of the origin of the environment of the robot
   * @return longitude coordinate of the origin of the environment of the robot
   */
  inline const double &lon0() const { return lon0_; }

  /**
   * Returns altitude coordinate of the origin of the environment of the robot
   * @return altitude coordinate of the origin of the environment of the robot
   */
  inline const double &alt0() const { return alt0_; }

  /**
   * Returns vector of origin coordinates of the environment of the robot
   * @return vector of origin coordinates of the environment of the robot
   */
  inline Eigen::Vector3d coord0() const { return {lat0_, lon0_, alt0_}; }

  /**
   * Returns pressure at sea level
   * @return pressure at sea level
   */
  inline const double &atm() const { return atm_; }

  /**
   * Returns density of water
   * @return density of water
   */
  inline const double &wd() const { return wd_; }

  /**
   * Returns switching altitude between descent/exploration mode
   * @return switching altitude between descent/exploration mode
   */
  inline const double &alt_switch() const { return alt_switch_; }

private:

  /**
   * Callback function for the dynamic reconfigure client
   * @param config parameter config
   */
  void callback(const forssea_msgs::EnvironmentConfig &config);

  double g_ = 9.81; /**< Gravity value*/
  double lat0_ = 0; /**< Latitude of the origin of the local frame*/
  double lon0_ = 0; /**< Longitude of the origin of the local frame*/
  double alt0_ = 0; /**< Altitude of the origin of the local frame*/
  double atm_ = 101325; /**< Atmospheric pressure*/
  double wd_ = 1030; /**< Sea water density*/
  double alt_switch_ = 10; /**< Switching altitude between descent/exploration mode*/
  DynamicParametersClient<forssea_msgs::EnvironmentConfig> client_; /**< Dynamic reconfigure client*/
};

}

#endif //SRC_ENVIRONMENT_HPP
