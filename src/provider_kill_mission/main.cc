// ROS includes
#include "provider_kill_mission_node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "provider_kill_mission_node");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    provider_kill_mission::ProviderKillMissionNode pkm(nh);
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        pkm.poll_kill_mission();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
