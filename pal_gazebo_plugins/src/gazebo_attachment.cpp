/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/
#include <pal_gazebo_plugins/gazebo_attachment.h>
#include <ignition/math.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>

namespace gazebo
{
void GazeboAttachment::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model_ = _parent;
  this->world_ = _parent->GetWorld();
  this->connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboAttachment::OnUpdate, this, std::placeholders::_1));

  if (_sdf->HasElement("target_model_name"))
  {
    this->target_model_name_ = _sdf->Get<std::string>("target_model_name");
  }
  else
  {
    ROS_ERROR_STREAM("Didn't find target_model_name");
    return;
  }
  if (_sdf->HasElement("target_link_name"))
  {
    this->target_link_name_ = _sdf->Get<std::string>("target_link_name");
  }
  else
  {
    ROS_ERROR_STREAM("Didn't find target_link_name");
    return;
  }
  if (_sdf->HasElement("local_link_name"))
  {
    this->local_link_name_ = _sdf->Get<std::string>("local_link_name");
  }
  else
  {
    ROS_ERROR_STREAM("Didn't find local_link_name");
    return;
  }

  if (_sdf->HasElement("pose"))
  {
    this->pose_ = _sdf->Get<ignition::math::Pose3d>("pose");
  }
  else
  {
    ROS_ERROR_STREAM("Didn't find pose");
    return;
  }
}



void gazebo::GazeboAttachment::createJoint()
{
  const auto p = this->target_link_->WorldPose();
  ignition::math::Pose3d target_link_pose =
      ignition::math::Pose3d(p.Pos().X(), p.Pos().Y(), p.Pos().Z(), p.Rot().W(), p.Rot().X(), p.Rot().Y(), p.Rot().Z());

  ignition::math::Pose3d target_pose = ignition::math::Pose3d(
      target_link_pose.Pos() + target_link_pose.Rot().RotateVector(this->pose_.Pos()),
      target_link_pose.Rot() * this->pose_.Rot());
  this->model_->SetLinkWorldPose(target_pose, this->local_link_);

  physics::JointPtr joint =
      this->world_->Physics()->CreateJoint("revolute", this->model_);
  joint->Attach(this->local_link_, this->target_link_);
  joint->Load(this->local_link_, this->target_link_, ignition::math::Pose3d());
  joint->SetModel(this->model_);
  joint->SetUpperLimit(0, 0);
  joint->SetLowerLimit(0, 0);
  joint->Init();
}

void GazeboAttachment::OnUpdate(const common::UpdateInfo &)
{
  ROS_INFO_STREAM_THROTTLE(5.0, "Attempting to attach " << this->target_model_name_
                                                        << "::" << this->target_link_name_
                                                        << " to " << this->local_link_name_);
  this->target_model_ = this->world_->ModelByName(this->target_model_name_);
  if (!this->target_model_.get())
  {
    ROS_ERROR_STREAM_DELAYED_THROTTLE(5.0, "gazebo_attachment plugin got non-existing target_model_name: "
                                               << this->target_model_name_);
    return;
  }

  this->target_link_ = this->target_model_->GetLink(this->target_link_name_);
  if (!this->target_link_.get())
  {
    ROS_ERROR_STREAM_DELAYED_THROTTLE(5.0, "gazebo_attachment plugin got non-existing target_link_name: "
                                               << this->target_link_name_);
    return;
  }

  this->local_link_ = this->model_->GetLink(this->local_link_name_);
  if (!this->local_link_.get())
  {
    ROS_ERROR_STREAM_DELAYED_THROTTLE(5.0, "gazebo_attachment plugin got non-existing local_link_name: "
                                               << this->local_link_name_);
    return;
  }

  createJoint();
  this->connection_.reset();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboAttachment)
}  // namespace gazebo
