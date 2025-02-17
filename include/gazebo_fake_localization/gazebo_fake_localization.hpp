/**
 * Carl Cort (ccort6)
 * 11/05/24
 * Modified from main branch to support renaming of tf child topic name and a derived 
 * class, DirectPoseGazeboFakeLocalization, which publishes the tf from the fixed 
 * gazebo frame -> robot base_footprint instead of the map -> odom tf. This provides
 * direct access to the gt_pose (robot pose in the gazebo frame).
 */

#ifndef GAZEBO_FAKE_LOCALIZATION_H
#define GAZEBO_FAKE_LOCALIZATION_H

#include <ros/ros.h>
#include <ros/topic.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>

class GazeboFakeLocalization
{
protected:
  ros::NodeHandle nh_, pnh_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_pub_;
  ros::Subscriber state_sub_; // Only active if pub_freq <= 0
  ros::Timer timer_; // Only active if pub_freq > 0
  ros::Publisher pose_pub_; // Only active if pub_pose_ is true

  std::string output_frame_id_, odom_frame_id_, base_frame_id_, gazebo_frame_id_, publish_frame_id;
  
  std::string model_name_;
  
  gazebo_msgs::ModelStates::ConstPtr states_;
  
  geometry_msgs::TransformStamped::Ptr Tr_m_;

  bool zero_z_;
  bool pub_gt_pose_;
  int throttle_gt_poses_;
  int gt_pose_throttle_count_;
  ros::Time last_update_time_;
  
public:
  GazeboFakeLocalization(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  
  virtual ~GazeboFakeLocalization() = default;

  virtual void updateTransform(geometry_msgs::TransformStamped::Ptr tr_m);

  geometry_msgs::TransformStamped::Ptr getModelTransform(const gazebo_msgs::ModelStates::ConstPtr& states);
  
  void updateTransform(const gazebo_msgs::ModelStates::ConstPtr& states);
  
  void updateTransform();
  
  void stateCB(const gazebo_msgs::ModelStates::ConstPtr& states);

  void timerCB(const ros::TimerEvent&);

  void init();
};

#endif // GAZEBO_FAKE_LOCALIZATION_H
