#include <ros/ros.h>

#include "twist_jogger.h"

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "twist_jogger_node");

    TwistJogger jogger;
    //jogger.spin();
    ros::spin();

    return 0;
}
