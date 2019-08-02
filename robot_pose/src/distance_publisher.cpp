/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <pal_statistics/pal_statistics_macros.h>

namespace pal
{
class DistanceTravelledPublisher
{
public:
  DistanceTravelledPublisher()
  {
    distance_travelled_ = 0.0;
    distance_pub_ = nh_.advertise<std_msgs::Float64>("distance_travelled", 1, true);
    pose_sub_ = nh_.subscribe("/mobile_base_controller/odom", 1000, &DistanceTravelledPublisher::poseCb, this);
    statistics_timer_ = nh_.createTimer(
        ros::Duration(1.0), boost::bind(&DistanceTravelledPublisher::doPublish, this));
    REGISTER_VARIABLE(DEFAULT_STATISTICS_TOPIC, "pal.distance_travelled",
                      &distance_travelled_, &registration_raii_);


    std_msgs::Float64 msg;
    msg.data = distance_travelled_;
    distance_pub_.publish(msg);
  }

  void doPublish()
  {
    PUBLISH_STATISTICS(DEFAULT_STATISTICS_TOPIC);
  }

protected:
  void poseCb(const nav_msgs::OdometryConstPtr &odom)
  {
    if (last_odom_.get())
    {
      const double dx = odom->pose.pose.position.x - last_odom_->pose.pose.position.x;
      const double dy = odom->pose.pose.position.y - last_odom_->pose.pose.position.y;
      distance_travelled_ += sqrt(dx * dx + dy * dy);
      std_msgs::Float64 msg;
      msg.data = distance_travelled_;
      distance_pub_.publish(msg);
    }
    last_odom_ = odom;
  }

  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Publisher distance_pub_;
  ros::Timer statistics_timer_;
  nav_msgs::OdometryConstPtr last_odom_;
  double distance_travelled_;
  pal_statistics::RegistrationsRAII registration_raii_;
};

}  // namespace pal


int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_publisher");
  pal::DistanceTravelledPublisher dtp;
  ros::spin();
}
