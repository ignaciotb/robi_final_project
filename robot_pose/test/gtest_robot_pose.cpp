/*
  @file gtest_robot_pose.cpp

  @author Procópio Stein

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <std_msgs/Float64.h>

namespace pal
{

TEST(RobotPose, test)
{
  ros::NodeHandle nh;

  ros::Duration(2.0).sleep();  // Allow subscribers to connect

  ROS_INFO("wait for /robot_pose");
  boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> robot_pose_shared_ptr;
    robot_pose_shared_ptr = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(
        "/robot_pose", ros::Duration(1.0));

  EXPECT_TRUE(robot_pose_shared_ptr != NULL);
  EXPECT_EQ(robot_pose_shared_ptr->pose.pose.position.x, 10);
  EXPECT_EQ(robot_pose_shared_ptr->pose.pose.position.y, 0);

}
}  // namespace pal

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_pose_test");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
