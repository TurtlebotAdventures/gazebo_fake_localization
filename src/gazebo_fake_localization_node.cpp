#include "gazebo_fake_localization/gazebo_fake_localization.hpp"

int main(int argc, char** argv) {
    ros::init(argc,argv,"gazebo_fake_localization");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    GazeboFakeLocalization pub(nh,pnh);
    pub.init();

    ros::spin();
}