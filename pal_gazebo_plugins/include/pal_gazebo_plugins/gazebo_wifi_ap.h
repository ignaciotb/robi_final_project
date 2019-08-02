/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef GAZEBO_WIFI_AP_CPP
#define GAZEBO_WIFI_AP_CPP

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>

namespace gazebo {

  class Entity;

  class GazeboWifiAP : public ModelPlugin {

  public:
    GazeboWifiAP();
    ~GazeboWifiAP();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  protected:
    virtual void UpdateChild();

  /// \brief ROS callback queue thread
  private: void RosQueueThread();

  /// \brief: thread out Load function with
  /// with anything that might be blocking.
  private: void DeferredLoad();

  private:

    physics::WorldPtr world_;
    physics::ModelPtr parent_;
    event::ConnectionPtr update_connection_;

    std::string robot_model_name_;
    std::string topic_;
    std::string essid_;
    double max_value_;
    double min_value_;
    double max_range_;
    double sigma_;

    gazebo::physics::ModelPtr robot_ptr_;

    double      update_rate_;
    boost::thread deferredLoadThread_;

    // ROS stuff
    ros::NodeHandle* rosNode_;
    ros::CallbackQueue rosQueue_;
    boost::thread callbackQueeuThread_;
    ros::Publisher pubWifiMsg_;

    // Controls stuff
    ros::Time lastUpdateTime_;

    // Mutex
    boost::mutex mutex_;
  };

}

#endif // GAZEBO_WIFI_AP_CPP
