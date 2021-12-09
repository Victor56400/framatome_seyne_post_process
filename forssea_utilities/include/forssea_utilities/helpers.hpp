#ifndef FUTILITIES_HELPERS_HPP
#define FUTILITIES_HELPERS_HPP

#include <sstream>
#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>


namespace forssea_utilities
{

/**
 * Looks for a parameter in the parameter server (default value version)
 * @tparam T1 type of the parameter
 * @tparam T2 type of the default value
 * @param param_name name of the parameter to look for
 * @param parameter container for the parameter
 * @param default_value default value of the parameter
 * @param has_default true if a default value is given by the user, false otherwise
 * @return true if the parameter has been found or the default value has been used, false otherwise
 */
template<typename T1, typename T2>
bool getParam(const std::string &param_name, T1 &parameter, const T2 &default_value, bool has_default = true)
{
  std::string param_path;
  if (ros::param::search(param_name, param_path))
  {
    ros::param::get(param_path, parameter);
    ROS_DEBUG_STREAM("Node found the parameter `" << param_name << "` in the parameter server : " << parameter);
    return true;
  } else
  {
    std::ostringstream ss;
    ss << "Node could not find the parameter `" << param_name << "` in the parameter server. ";
    if (has_default)
    {
      parameter = default_value;
      ss << "Setting default value: " << parameter;
      ROS_WARN("%s", ss.str().c_str());
      return true;
    } else
    {
      ss << "No default value set. This node might crash.";
      ROS_WARN("%s", ss.str().c_str());
      return false;
    }
  }
}

/**
 * Looks for a parameter in the parameter server
 * @tparam T type of the parameter
 * @param param_name name of the parameter
 * @param parameter container of the parameter
 * @return true if the parameter has been found, false otherwise
 */
template<typename T>
bool getParam(const std::string &param_name, T &parameter)
{
  return getParam(param_name, parameter, T(), false);
}

/**
 * std::vector display function
 * @tparam T type of the vector's elements
 * @param out output stream
 * @param v vector to display
 * @return reference to the output stream
 */
template<typename T>
std::ostream &operator<<(std::ostream &out, const std::vector<T> &v)
{
  if (!v.empty())
  {
    out << '[';
    std::copy(v.begin(), v.end() - 1, std::ostream_iterator<T>(out, ", "));
    out << v.back() << "]";
  }
  return out;
}

/**
 * boost::container::vector display function
 * @tparam T type of the vector's elements
 * @param out output stream
 * @param v vector to display
 * @return reference to the output stream
 */
template<typename T>
std::ostream &operator<<(std::ostream &out, const boost::container::vector<T> &v)
{
  if (!v.empty())
  {
    out << '[';
    std::copy(v.begin(), v.end() - 1, std::ostream_iterator<T>(out, ", "));
    out << v.back() << "]";
  }
  return out;
}

/**
 * Adds a value to each element of a vector
 * @tparam T type of the elements of the vector
 * @param v vector
 * @param add value to add
 * @return reference to the vector
 */
template<typename T>
std::vector<T> &operator+=(std::vector<T> &v, const T &add)
{
  if (!v.empty())
  {
    for (auto &i : v) i += add;
  }
  return v;
}

/**
 * Mod each element of a vector by a value
 * @tparam T type of the vector's elements
 * @param v vector
 * @param mod value to mod
 * @return reference to the vector
 */
template<typename T>
std::vector<T> &operator%=(std::vector<T> &v, const T &mod)
{
  if (!v.empty())
  {
    for (auto &i : v) i %= mod;
  }
  return v;
}

/**
 * Returns the sign of a number
 * @tparam T type of the number
 * @param val number
 * @return +1 if number > 0, 0 if number = 0, -1 if number < 0
 */
template<typename T>
inline int sgn(const T &val)
{
  return (T(0) < val) - (val < T(0));
}

/**
 * Check that a value is in a given semi open interval
 * @tparam T1 type of the value
 * @tparam T2 type of the interval bounds
 * @param val value
 * @param min interval min bound
 * @param max interval max bound
 * @return true if min <= val < max, false otherwise
 */
template<typename T1, typename T2>
inline bool inRange(const T1 &val, const T2 &min, const T2 &max)
{
  return min <= val and val < max;
}

/**
 * Returns the angle equivalent to the input value, but in range [-pi, pi]
 * @param value input angle
 * @return angle equivalent to the input value, but in range [-pi, pi]
 */
inline double angle(const double &value)
{
  return std::fmod(value + M_PI * sgn(value), 2 * M_PI) - M_PI * sgn(value);
}

inline void getRPY(const geometry_msgs::Quaternion &msg, double &roll, double &pitch, double &yaw)
{
  tf::Quaternion q(msg.x, msg.y, msg.z, msg.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}
}

#endif
