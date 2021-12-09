#include <forssea_utilities/Environment.hpp>


namespace forssea_utilities {

Environment::Environment(const ros::NodeHandle &nh) : client_(nh) {
  client_.start(boost::bind(&Environment::callback, this, _1));
}

void Environment::callback(const forssea_msgs::EnvironmentConfig &config) {
  g_ = config.gravity;
  atm_ = config.zero_pressure;
  wd_ = config.water_density;
  lat0_ = config.target_latitude;
  lon0_ = config.target_longitude;
  alt0_ = config.target_altitude;
  alt_switch_ = config.min_target_altitude;
  ROS_DEBUG_STREAM("Environment variables successfully updated.");
}
}
