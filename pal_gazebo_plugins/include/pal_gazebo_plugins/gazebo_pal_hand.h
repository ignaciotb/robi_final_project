/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef GAZEBO_PAL_HAND_H
#define GAZEBO_PAL_HAND_H

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <control_toolbox/pid.h>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboPalHand : public ModelPlugin {

    public:
      GazeboPalHand();
      ~GazeboPalHand();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
      virtual void UpdateChild();

    private:

      physics::WorldPtr world;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      std::string finger_joint_name_;
      std::string finger_1_joint_name_;
      std::string finger_2_joint_name_;
      std::string finger_3_joint_name_;

      physics::JointPtr joints[4];

      std::string robot_namespace_;

  };

}

#endif // GAZEBO_PAL_HAND_H
