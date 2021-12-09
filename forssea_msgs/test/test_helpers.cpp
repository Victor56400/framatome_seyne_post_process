#include <forssea_msgs/helpers.hpp>
#include <gtest/gtest.h>
#include <iostream>

using namespace forssea_msgs;

TEST(RobotStateApi, state2pos)
{
    double x(1.0000001), y(2.356468126), z(-89526.1256654);
    RobotStatePtr state(new RobotState());
    state->p.x = x;
    state->p.y = y;
    state->p.z = z;
    Eigen::Vector3d v1 = state2pos(state);
    Eigen::Vector3d v2(x, y, z);
    EXPECT_EQ(v1, v2);
}

TEST(RobotStateApi, state2vel)
{
    double x(1.0000001), y(2.356468126), z(-89526.1256654);
    RobotStatePtr state(new RobotState());
    state->v.x = x;
    state->v.y = y;
    state->v.z = z;
    Eigen::Vector3d v1 = state2vel(state);
    Eigen::Vector3d v2(x, y, z);
    EXPECT_EQ(v1, v2);
}

TEST(RobotStateApi, state2acc)
{
    double x(1.0000001), y(2.356468126), z(-89526.1256654);
    RobotStatePtr state(new RobotState());
    state->a.x = x;
    state->a.y = y;
    state->a.z = z;
    Eigen::Vector3d v1 = state2acc(state);
    Eigen::Vector3d v2(x, y, z);
    EXPECT_EQ(v1, v2);
}

TEST(RobotStateApi, state2eul)
{
    double x(1.0000001), y(2.356468126), z(-89526.1256654);
    RobotStatePtr state(new RobotState());
    state->e.x = x;
    state->e.y = y;
    state->e.z = z;
    Eigen::Vector3d v1 = state2eul(state);
    Eigen::Vector3d v2(x, y, z);
    EXPECT_EQ(v1, v2);
}

TEST(RobotStateApi, state2rot)
{
    double x(1.0000001), y(2.356468126), z(-89526.1256654);
    RobotStatePtr state(new RobotState());
    state->w.x = x;
    state->w.y = y;
    state->w.z = z;
    Eigen::Vector3d v1 = state2rot(state);
    Eigen::Vector3d v2(x, y, z);
    EXPECT_EQ(v1, v2);
}

TEST(RobotStateApi, state2mat)
{
    RobotStatePtr state(new RobotState());
    state->e.x = 0;
    state->e.y = 0;
    state->e.z = 0;
    Eigen::Matrix3d m1 = state2mat(state);
    Eigen::Matrix3d m2;
    m2 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    EXPECT_EQ(m1(0, 0), m2(0, 0));
    EXPECT_EQ(m1(1, 1), m2(1, 1));
    EXPECT_EQ(m1(2, 2), m2(2, 2));
    state->e.x = M_PI;
    state->e.y = 0;
    state->e.z = 0;
    m1 = state2mat(state);
    m2 << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    EXPECT_EQ(m1(0, 0), m2(0, 0));
    EXPECT_EQ(m1(1, 1), m2(1, 1));
    EXPECT_EQ(m1(2, 2), m2(2, 2));
    state->e.x = 0;
    state->e.y = M_PI;
    state->e.z = 0;
    m1 = state2mat(state);
    m2 << -1, 0, 0, 0, 1, 0, 0, 0, -1;
    EXPECT_EQ(m1(0, 0), m2(0, 0));
    EXPECT_EQ(m1(1, 1), m2(1, 1));
    EXPECT_EQ(m1(2, 2), m2(2, 2));
    state->e.x = 0;
    state->e.y = 0;
    state->e.z = M_PI;
    m1 = state2mat(state);
    m2 << -1, 0, 0, 0, -1, 0, 0, 0, 1;
    EXPECT_EQ(m1(0, 0), m2(0, 0));
    EXPECT_EQ(m1(1, 1), m2(1, 1));
    EXPECT_EQ(m1(2, 2), m2(2, 2));
}

TEST(RobotStateApi, pos2state)
{
    RobotStatePtr state(new RobotState());
    Eigen::Vector3d pos(1, 2, 3);
    pos2state(pos, state);
    EXPECT_EQ(pos.x(), state->p.x);
    EXPECT_EQ(pos.y(), state->p.y);
    EXPECT_EQ(pos.z(), state->p.z);
}

TEST(RobotStateApi, vel2state)
{
    RobotStatePtr state(new RobotState());
    Eigen::Vector3d vel(1, 2, 3);
    vel2state(vel, state);
    EXPECT_EQ(vel.x(), state->v.x);
    EXPECT_EQ(vel.y(), state->v.y);
    EXPECT_EQ(vel.z(), state->v.z);
}

TEST(RobotStateApi, acc2state)
{
    RobotStatePtr state(new RobotState());
    Eigen::Vector3d acc(1, 2, 3);
    acc2state(acc, state);
    EXPECT_EQ(acc.x(), state->a.x);
    EXPECT_EQ(acc.y(), state->a.y);
    EXPECT_EQ(acc.z(), state->a.z);
}

TEST(RobotStateApi, eul2state)
{
    RobotStatePtr state(new RobotState());
    Eigen::Vector3d eul(1, 2, 3);
    eul2state(eul, state);
    EXPECT_EQ(eul.x(), state->e.x);
    EXPECT_EQ(eul.y(), state->e.y);
    EXPECT_EQ(eul.z(), state->e.z);
}

TEST(RobotStateApi, rot2state)
{
    RobotStatePtr state(new RobotState());
    Eigen::Vector3d rot(1, 2, 3);
    rot2state(rot, state);
    EXPECT_EQ(rot.x(), state->w.x);
    EXPECT_EQ(rot.y(), state->w.y);
    EXPECT_EQ(rot.z(), state->w.z);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
