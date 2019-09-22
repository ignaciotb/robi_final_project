/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef GAZEBO_PAL_HEY5_H
#define GAZEBO_PAL_HEY5_H


// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <control_toolbox/pid.h>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboPalHey5 : public ModelPlugin {

    public:
      GazeboPalHey5();
      ~GazeboPalHey5();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
      virtual void UpdateChild();

    private:

      physics::WorldPtr world;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      std::string actuated_joint_name_;

      std::vector<std::string> virtual_joint_names_;

      physics::JointPtr actuated_joint_;

      std::vector<physics::JointPtr> virtual_joints_;
      typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
      std::vector<PidPtr> pids_;

      std::vector<double> scale_factors_;

      ignition::math::Angle actuator_angle_;

      std::string robot_namespace_;

  };

}

#endif // GAZEBO_PAL_HEY5_H
