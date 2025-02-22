/**
 * Carl Cort (ccort6)
 * 11/05/24
 * Modified from main branch to support renaming of tf child topic name and a derived 
 * class, DirectPoseGazeboFakeLocalization, which publishes the tf from the fixed 
 * gazebo frame -> robot base_footprint instead of the map -> odom tf. This provides
 * direct access to the gt_pose (robot pose in the gazebo frame).
 */

#include "gazebo_fake_localization/gazebo_fake_localization.hpp"

GazeboFakeLocalization::GazeboFakeLocalization(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
  nh_(nh),
  pnh_(pnh),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  odom_frame_id_ = "odom";
  base_frame_id_ = "base_footprint";
  gazebo_frame_id_ = "map";
  model_name_ = "mobile_base";

  zero_z_ = false;
}

// tf naming convention: Ttarget_source refers to the transform of the target frame in the source reference frame
//                       i.e. the tf that defines source -> target.
//                       r = robot (base_frame_id_), m = map (gazebo_frame_id_), o = odom (odom_frame_id_)
//    ex.) Tr_m is the transform of the robot in the map reference frame. i.e. it is the ground truth robot pose
void GazeboFakeLocalization::updateTransform(geometry_msgs::TransformStamped::Ptr Tr_m)
{
  if(Tr_m)
  {
    if (braodcast_tf_) {
      try
      {
        geometry_msgs::TransformStamped To_r = tf_buffer_.lookupTransform(base_frame_id_, odom_frame_id_, Tr_m->header.stamp, ros::Duration(.1));
        geometry_msgs::TransformStamped To_m;
        
        // Applies the Tr_m (robot in map frame i.e. gazebo_world -> robot) transform to the
        // To_r (odom frame in robot frame i.e. robot -> odom) to get To_m (odom in map frame i.e. map -> odom) tf
        tf2::doTransform(To_r, To_m, *Tr_m);
        
        To_m.child_frame_id = output_frame_id_;
        
        tf_pub_.sendTransform(To_m);
        last_update_time_ = To_m.header.stamp;
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("Gazebo_Gake_Localization: %s",ex.what());
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

geometry_msgs::TransformStamped::Ptr GazeboFakeLocalization::getModelTransform(const gazebo_msgs::ModelStates::ConstPtr& states)
{
  geometry_msgs::TransformStamped::Ptr t;
  
  std::vector<std::string>::const_iterator iter = std::find(states->name.begin(), states->name.end(), model_name_);
  
  if( iter != states->name.end() )
  {
    int index = std::distance(states->name.begin(), iter);
    const geometry_msgs::Pose& robot_state = states->pose[index];
    
    t = boost::make_shared<geometry_msgs::TransformStamped>();
    
    t->header.stamp = ros::Time::now();
    t->header.frame_id = gazebo_frame_id_;
    t->child_frame_id = base_frame_id_;
    t->transform.translation.x = robot_state.position.x;
    t->transform.translation.y = robot_state.position.y;
    if(!zero_z_)
      t->transform.translation.z = robot_state.position.z;
    t->transform.rotation = robot_state.orientation;
    
  }
  
  return t;
}


void GazeboFakeLocalization::updateTransform(const gazebo_msgs::ModelStates::ConstPtr& states )
{
  if(states)
  {
    geometry_msgs::TransformStamped::Ptr t = getModelTransform(states);
    
    if(t && t->header.stamp > last_update_time_)
    {
      updateTransform(t);
    }
  }
}

void GazeboFakeLocalization::updateTransform()
{
  updateTransform(states_);
}

void GazeboFakeLocalization::stateCB(const gazebo_msgs::ModelStates::ConstPtr& states )
{
  states_ = states;
}

void GazeboFakeLocalization::timerCB(const ros::TimerEvent&)
{
    updateTransform();
}


void GazeboFakeLocalization::init()
{
  braodcast_tf_ = true;
  pnh_.getParam("broadcast_tf", braodcast_tf_);
  pub_gt_pose_ = false;
  pnh_.getParam("pub_gt_pose", pub_gt_pose_);
  if (!braodcast_tf_) {
    ROS_WARN("Gazebo Fake Localization configured to not broadcasting tf, publishing gt_pose as topic instead: %d", braodcast_tf_);
  }
  bool use_odom=false;
  pnh_.getParam("use_odom", use_odom);
  
  if(use_odom)
  {
    nav_msgs::Odometry::ConstPtr odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("odom", nh_);
    odom_frame_id_ = odom_msg->header.frame_id;
    base_frame_id_ = odom_msg->child_frame_id;
  }
  else
  {
    pnh_.getParam("base_frame_id", base_frame_id_);
    pnh_.getParam("odom_frame_id", odom_frame_id_);
  }
  
  // If not used, should just be the same as the odom_frame_id, which is how the original implementation worked
  pnh_.param("output_frame_id", output_frame_id_, odom_frame_id_);

  pnh_.getParam("model_name", model_name_);

  pnh_.getParam("zero_z", zero_z_);
  if (pub_gt_pose_)
  {
    // intended for data saving purposes since rospy tf_buffer was struggling with sim resets in nav_scripts
    // As such, publishes map -> robot pose directly, not map -> odom tf (which is the tf that is broadcast normally)
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_frame_id_, 1);
    throttle_gt_poses_ = 250;
    pnh_.getParam("throttle_gt_poses", throttle_gt_poses_);
    gt_pose_throttle_count_ = 0;
  }
  double pub_freq = -1;
  pnh_.getParam("freq", pub_freq);
  
  if(pub_freq <=0)
  {
    state_sub_ = nh_.subscribe("/gazebo/model_states", 1, &GazeboFakeLocalization::updateTransform, this);
  }
  else
  {
    state_sub_ = nh_.subscribe("/gazebo/model_states", 1, &GazeboFakeLocalization::stateCB, this);
    
    timer_ = nh_.createTimer(ros::Duration(1.0/pub_freq), &GazeboFakeLocalization::timerCB, this);
  }
  
  

}