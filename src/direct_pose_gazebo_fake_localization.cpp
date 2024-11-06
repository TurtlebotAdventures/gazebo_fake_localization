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

// tf naming convention: Ttarget_source refers to the transform of the target frame in the source reference frame
//                       i.e. the tf that defines source -> target.
//                       r = robot (base_frame_id_), m = map (gazebo_frame_id_), o = odom (odom_frame_id_)
//    ex.) Tr_m is the transform of the robot in the map reference frame. i.e. it is the ground truth robot pose
void DirectPoseGazeboLocalization::updateTransform(geometry_msgs::TransformStamped::Ptr Tr_m) {
    if(Tr_m)
    {
        try
        {
            Tr_m->child_frame_id = output_frame_id_;
            
            tf_pub_.sendTransform(*Tr_m);
            last_update_time_ = Tr_m->header.stamp;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
        }   
    }
}