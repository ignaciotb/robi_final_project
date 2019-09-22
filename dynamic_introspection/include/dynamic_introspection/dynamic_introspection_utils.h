/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _DYNAMIC_INTROSPECTION_UTILS_
#define _DYNAMIC_INTROSPECTION_UTILS_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pal_statistics_msgs/Statistics.h>
#include <Eigen/Dense>

namespace dynamic_introspection
{
class IntrospectionBagReader
{
public:
  IntrospectionBagReader();

  IntrospectionBagReader(const std::string &packageName, const std::string &bagFileName,
                         const std::string introspection_topic_name);

  IntrospectionBagReader(const std::string &bagFileName,
                         const std::string introspection_topic_name);

  virtual ~IntrospectionBagReader();

  void readBag(rosbag::Bag &bag);

  unsigned int getNumberMessages();

  void getVariable(const std::string &variableId, std::vector<bool> &value,
                   const bool throw_not_existing = true);

  void getVariable(const std::string &variableId, std::vector<double> &value,
                   const bool throw_not_existing = true);

  void getVariable(const std::string &variableId, std::vector<Eigen::Vector3d> &value,
                   const bool throw_not_existing = true);

  void getVariable(const std::vector<std::string> &variableId,
                   std::vector<Eigen::Vector3d> &value, const bool throw_not_existing);

  void getVariable(const std::string &variable_id, std::vector<Eigen::Quaterniond> &value,
                   const bool throw_not_existing = true);

  //  void getVariable(const std::vector<std::string> &names, std::vector<Eigen::VectorXd>
  //  &value,
  //                   const bool throw_not_existing = true);


  void addMsg(const pal_statistics_msgs::StatisticsConstPtr s);

  std::vector<std::map<std::string, int>> doubleNameMap_;

private:
  bool first_msg_;
  unsigned int counter_;
  std::string introspection_topic_name_;

  //  unsigned int n_messages_;

  std::vector<std::vector<double>> doubleValues_;
};
}

#endif
