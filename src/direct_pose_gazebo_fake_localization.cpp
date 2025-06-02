/**
 * Carl Cort (ccort6)
 * 11/05/24
 * Modified GazeboFakeLocalization which publishes the tf from the fixed 
 * gazebo frame -> robot base_footprint instead of the map -> odom tf. This provides
 * direct access to the gt_pose (robot pose in the gazebo frame).
 */

#include "gazebo_fake_localization/direct_pose_gazebo_fake_localization.hpp"

DirectPoseGazeboLocalization::DirectPoseGazeboLocalization(ros::NodeHandle& nh, ros::NodeHandle& pnh) : GazeboFakeLocalization(nh, pnh)
{}

// tf naming convention: Target_source refers to the transform of the target frame in the source reference frame
//                       i.e. the tf that defines source -> target.
//                       r = robot (base_frame_id_), m = map (gazebo_frame_id_), o = odom (odom_frame_id_)
//    ex.) Tr_m is the transform of the robot in the map reference frame. i.e. it is the ground truth robot pose
void DirectPoseGazeboLocalization::updateTransform(geometry_msgs::TransformStamped::Ptr Tr_m) {
    if(Tr_m)
    {
        if (braodcast_tf_) {
            try
            {
                Tr_m->child_frame_id = output_frame_id_;
                Tr_m->header.stamp += ros::Duration(0.002); // to avoid tf2 warnings
                if(last_time_ < Tr_m->header.stamp)
                {
                    tf_pub_.sendTransform(*Tr_m);
                    last_time_ = Tr_m->header.stamp;
                }
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("DirectPoseGazeboLocalization: %s",ex.what());
            }
        }
        if (pub_gt_pose_ && gt_pose_throttle_count_ == 0)
        {
            // intended for data saving purposes since rospy tf_buffer was struggling with sim resets in nav_scripts
            // As such, publishes map -> robot pose directly, not map -> odom tf (which is the tf that is broadcast normally)
            geometry_msgs::PoseStamped pose;
            pose.header = Tr_m->header;
            pose.pose.position.x = Tr_m->transform.translation.x;
            pose.pose.position.y = Tr_m->transform.translation.y;
            pose.pose.position.z = Tr_m->transform.translation.z;
            pose.pose.orientation = Tr_m->transform.rotation;
            pose_pub_.publish(pose);
        }
        gt_pose_throttle_count_ = (gt_pose_throttle_count_ + 1) % throttle_gt_poses_;
    }
}