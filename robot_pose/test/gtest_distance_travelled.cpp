/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

namespace pal
{
TEST(DistanceTravelled, test)
{
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/mobile_base_controller/odom", 1);
  ros::Duration(0.5).sleep();  // Allow subscribers to connect
  nav_msgs::Odometry msg;
  msg.pose.pose.orientation.w = 1.0;

  msg.pose.pose.position.x = 1234.0;  // First position shouldn't matter

  pub.publish(msg);

  std_msgs::Float64ConstPtr distance_msg;
  distance_msg = ros::topic::waitForMessage<std_msgs::Float64>("/distance_travelled",
                                                               ros::Duration(1.0));
  EXPECT_TRUE(distance_msg.get());
  EXPECT_DOUBLE_EQ(0.0, distance_msg->data);

  double expected_distance = 0.0;
  for (double step : { 0.01, -0.02, 0.03 })
  {
    for (int i = 0; i < 500; i++)
    {
      msg.pose.pose.position.x += step;
      msg.pose.pose.position.y -= 2 * step;
      pub.publish(msg);
      ros::spinOnce();
      expected_distance += sqrt((step * step) + (2 * step * 2 * step));
    }
    distance_msg = ros::topic::waitForMessage<std_msgs::Float64>("/distance_travelled",
                                                                 ros::Duration(1.0));
    EXPECT_NEAR(expected_distance, distance_msg->data, 0.001);
  }
}
}  // namespace pal

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_travelled_test");
  // first nodehandle created of an app must exist until the end of the life of the node
  // If not, you'll have funny stuff such as no logs printed
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
