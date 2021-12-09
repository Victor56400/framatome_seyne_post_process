#ifndef FMSGS_HELPERS_HPP
#define FMSGS_HELPERS_HPP

#include <forssea_msgs/RobotState.h>
#include <eigen3/Eigen/Dense>


namespace forssea_msgs {
Eigen::Vector3d state2pos(const forssea_msgs::RobotState::ConstPtr &msg);
Eigen::Vector3d state2vel(const forssea_msgs::RobotState::ConstPtr &msg);
Eigen::Vector3d state2acc(const forssea_msgs::RobotState::ConstPtr &msg);
Eigen::Vector3d state2eul(const forssea_msgs::RobotState::ConstPtr &msg);
Eigen::Vector3d state2rot(const forssea_msgs::RobotState::ConstPtr &msg);
Eigen::Vector3d state2ang(const forssea_msgs::RobotState::ConstPtr &msg);
Eigen::Matrix3d state2mat(const forssea_msgs::RobotState::ConstPtr &msg);
void pos2state(const Eigen::Vector3d &vec, forssea_msgs::RobotState::Ptr &msg);
void vel2state(const Eigen::Vector3d &vec, forssea_msgs::RobotState::Ptr &msg);
void acc2state(const Eigen::Vector3d &vec, forssea_msgs::RobotState::Ptr &msg);
void eul2state(const Eigen::Vector3d &vec, forssea_msgs::RobotState::Ptr &msg);
void rot2state(const Eigen::Vector3d &vec, forssea_msgs::RobotState::Ptr &msg);
void ang2state(const Eigen::Vector3d &vec, forssea_msgs::RobotState::Ptr &msg);
}

#endif // FMSGS_HELPERS_HPP
