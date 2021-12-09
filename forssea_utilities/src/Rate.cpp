#include <forssea_utilities/Rate.hpp>


namespace forssea_utilities {

void Rate::sleep() {
  mutex_.lock();
  rate_.sleep();
  mutex_.unlock();
}

double Rate::frequency() {
  return frequency_;
}

void Rate::callback(forssea_msgs::RateConfig &config, uint32_t level) {
  mutex_.lock();
  rate_ = ros::Rate(config.frequency);
  frequency_ = config.frequency;
  mutex_.unlock();
  if (callback_) callback_(frequency_);
}

void Rate::setCallback(const Rate::CallbackType &callback) {
  callback_ = callback;
}

}
