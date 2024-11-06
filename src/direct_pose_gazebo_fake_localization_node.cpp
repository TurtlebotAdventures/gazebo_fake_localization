#include "gazebo_fake_localization/direct_pose_gazebo_fake_localization.hpp"

int main(int argc, char** argv) {
    ros::init(argc,argv,"direct_pose_gazebo_fake_localization");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    DirectPoseGazeboLocalization pub(nh,pnh);
    pub.init();

    ros::spin();
}