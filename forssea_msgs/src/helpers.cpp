#include <forssea_msgs/helpers.hpp>


Eigen::Vector3d forssea_msgs::state2pos(const forssea_msgs::RobotState::ConstPtr &msg) {
  return {msg->p.x, msg->p.y, msg->p.z};
}

Eigen::Vector3d forssea_msgs::state2vel(const forssea_msgs::RobotState::ConstPtr &msg) {
  return {msg->v.x, msg->v.y, msg->v.z};
}

Eigen::Vector3d forssea_msgs::state2acc(const forssea_msgs::RobotState::ConstPtr &msg) {
  return {msg->a.x, msg->a.y, msg->a.z};
}

Eigen::Vector3d forssea_msgs::state2eul(const forssea_msgs::RobotState::ConstPtr &msg) {
  return {msg->e.x, msg->e.y, msg->e.z};
}

Eigen::Vector3d forssea_msgs::state2rot(const forssea_msgs::RobotState::ConstPtr &msg) {
  return {msg->w.x, msg->w.y, msg->w.z};
}

Eigen::Vector3d forssea_msgs::state2ang(const forssea_msgs::RobotState::ConstPtr &msg) {
  return {msg->d.x, msg->d.y, msg->d.z};
}

Eigen::Matrix3d forssea_msgs::state2mat(const forssea_msgs::RobotState::ConstPtr &msg) {
  Eigen::Quaterniond q = Eigen::AngleAxisd(msg->e.x, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(msg->e.y, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(msg->e.z, Eigen::Vector3d::UnitZ());
  return q.matrix();

}

void forssea_msgs::pos2state(const Eigen::Vector3d &vec, forssea_msgs::RobotState::Ptr &msg) {
  msg->p.x = vec(0);
  msg->p.y = vec(1);
  msg->p.z = vec(2);
}

void forssea_msgs::vel2state(const Eigen::Vector3d &vec, forssea_msgs::RobotState::Ptr &msg) {
  msg->v.x = vec(0);
  msg->v.y = vec(1);
  msg->v.z = vec(2);
}

void forssea_msgs::acc2state(const Eigen::Vector3d &vec, forssea_msgs::RobotState::Ptr &msg) {
  msg->a.x = vec(0);
  msg->a.y = vec(1);
  msg->a.z = vec(2);
}

void forssea_msgs::eul2state(const Eigen::Vector3d &vec, forssea_msgs::RobotState::Ptr &msg) {
  msg->e.x = vec(0);
  msg->e.y = vec(1);
  msg->e.z = vec(2);
}

void forssea_msgs::rot2state(const Eigen::Vector3d &vec, forssea_msgs::RobotState::Ptr &msg) {
  msg->w.x = vec(0);
  msg->w.y = vec(1);
  msg->w.z = vec(2);
}

void forssea_msgs::ang2state(const Eigen::Vector3d &vec, forssea_msgs::RobotState::Ptr &msg) {
  msg->d.x = vec(0);
  msg->d.y = vec(1);
  msg->d.z = vec(2);
}
