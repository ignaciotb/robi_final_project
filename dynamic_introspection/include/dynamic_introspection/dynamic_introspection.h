/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _DYNAMIC_INTROSPECTION_
#define _DYNAMIC_INTROSPECTION_
#include <pal_statistics/pal_statistics_macros.h>
#include <rosbag/bag.h>
#include <Eigen/Dense>

namespace pal_statistics
{
template <>
inline IdType customRegister(StatisticsRegistry &registry, const std::string &name,
                             const Eigen::Vector3d *variable, RegistrationsRAII *bookkeeping,
                             bool enabled)
{
  /// only one id is returned, unregistration should be done with RegistrationRAII
  /// or vairable name
  registry.registerVariable(name + "_X", &variable->x(), bookkeeping, enabled);
  registry.registerVariable(name + "_Y", &variable->y(), bookkeeping, enabled);
  return registry.registerVariable(name + "_Z", &variable->z(), bookkeeping, enabled);
}

template <>
inline IdType customRegister(StatisticsRegistry &registry, const std::string &name,
                             const Eigen::Vector2d *variable, RegistrationsRAII *bookkeeping,
                             bool enabled)
{
  /// only one id is returned, unregistration should be done with RegistrationRAII
  /// or vairable name
  registry.registerVariable(name + "_X", &variable->x(), bookkeeping, enabled);
  return registry.registerVariable(name + "_Y", &variable->y(), bookkeeping, enabled);
}


template <>
inline IdType customRegister(StatisticsRegistry &registry, const std::string &name,
                             const Eigen::Quaterniond *variable, RegistrationsRAII *bookkeeping,
                             bool enabled)
{
  /// only one id is returned, unregistration should be done with RegistrationRAII
  /// or vairable name
  registry.registerVariable(name + "_QX", &variable->coeffs().x(), bookkeeping, enabled);
  registry.registerVariable(name + "_QY", &variable->coeffs().y(), bookkeeping, enabled);
  registry.registerVariable(name + "_QZ", &variable->coeffs().z(), bookkeeping, enabled);
  return registry.registerVariable(name + "_QW", &variable->coeffs().w(), bookkeeping, enabled);
}

// All these are hacks to preserve an old API
namespace bag_hack
{
static boost::shared_ptr<rosbag::Bag> bag_;
inline void statisticsOpenBag(const std::string &filename)
{
  bag_.reset(new rosbag::Bag);
  bag_->open(filename, rosbag::bagmode::Write);
}

inline void statisticsCloseBag()
{
  bag_->close();
  bag_.reset();
}

inline void statististicsWriteDataToBag()
{
  bag_->write("/introspection_data/full", ros::Time::now(),
              getRegistry("/introspection_data")->createMsg());
}
}  // namespace bag_hack
} // namespace pal

#define OPEN_BAG(BAG_NAME)                                            \
   pal_statistics::bag_hack::statisticsOpenBag(BAG_NAME);               \

#define PUBLISH_DEBUG_DATA_BAG                                        \
   pal_statistics::bag_hack::statististicsWriteDataToBag();                \

#define CLOSE_BAG                                                     \
   pal_statistics::bag_hack::statisticsCloseBag();                      \

#endif
