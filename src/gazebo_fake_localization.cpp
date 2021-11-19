
#include <ros/ros.h>
#include <ros/topic.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>


#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>


class GazeboFakeLocalization
{
private:
  ros::NodeHandle nh_, pnh_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_pub_;
  ros::Subscriber state_sub_;
  ros::Timer timer_;
  
  std::string odom_frame_id_, base_frame_id_, gazebo_frame_id_;
  
  std::string model_name_;
  
  gazebo_msgs::ModelStates::ConstPtr states_;
  
  ros::Time last_update_time_;
  
public:

  GazeboFakeLocalization(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh),
    pnh_(pnh),
    tf_buffer_(),
    tf_listener_(tf_buffer_)
  {
    odom_frame_id_ = "odom";
    base_frame_id_ = "base_footprint";
    gazebo_frame_id_ = "map";
    model_name_ = "mobile_base";


  }
  
  
  void updateTransform(geometry_msgs::TransformStamped::Ptr tr_m)
  {
    if(tr_m)
    {
      try
      {
        geometry_msgs::TransformStamped to_r = tf_buffer_.lookupTransform(base_frame_id_,odom_frame_id_, tr_m->header.stamp, ros::Duration(.1));
        geometry_msgs::TransformStamped t_out;
        
        tf2::doTransform(to_r, t_out, *tr_m);
        
        t_out.child_frame_id = odom_frame_id_;
        
        tf_pub_.sendTransform(t_out);
        last_update_time_ = t_out.header.stamp;
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
      }   
    }

  }

  geometry_msgs::TransformStamped::Ptr getModelTransform(const gazebo_msgs::ModelStates::ConstPtr& states)
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
      t->transform.translation.z = robot_state.position.z;
      t->transform.rotation = robot_state.orientation;
      
    }
    
    return t;
  }
  
  
  void updateTransform(const gazebo_msgs::ModelStates::ConstPtr& states )
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
  
  void updateTransform()
  {
    updateTransform(states_);
  }
  
  void stateCB(const gazebo_msgs::ModelStates::ConstPtr& states )
  {
    states_ = states;
  }

  void timerCB(const ros::TimerEvent&)
  {
      updateTransform();
  }

  
  void init()
  {
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
    
    pnh_.getParam("model_name", model_name_);
    
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

};


int main(int argc, char** argv)
{
  ros::init(argc,argv,"gazebo_fake_localization");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  GazeboFakeLocalization pub(nh,pnh);
  pub.init();
  
  ros::spin();
}
