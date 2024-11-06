/**
 * Carl Cort (ccort6)
 * 11/05/24
 * Modified GazeboFakeLocalization which publishes the tf from the fixed 
 * gazebo frame -> robot base_footprint instead of the map -> odom tf. This provides
 * direct access to the gt_pose (robot pose in the gazebo frame).
 */

#ifndef DIRECT_POSE_GAZEBO_LOCALIZATION_H
#define DIRECT_POSE_GAZEBO_LOCALIZATION_H

#include "gazebo_fake_localization/gazebo_fake_localization.hpp"

class DirectPoseGazeboLocalization : public GazeboFakeLocalization
{
public:
  DirectPoseGazeboLocalization(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  void updateTransform(geometry_msgs::TransformStamped::Ptr Tr_m) override;
};

#endif // DIRECT_POSE_GAZEBO_LOCALIZATION_H
