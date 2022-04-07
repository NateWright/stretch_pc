#include <ros/ros.h>

#include "pc.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_translation");
    ros::NodeHandle n;

    std::string topic;
    float precision;
    if (!n.getParam("/stretch_pc/pointCloudTopic", topic)) {
        ROS_DEBUG("stretch_pc: No pointcloud topic found\n");
        return (EXIT_FAILURE);
    }
    if (!n.getParam("/stretch_pc/precision", precision)) {
        ROS_DEBUG("stretch_pc: No precision found\n");
        return (EXIT_FAILURE);
    }

    pc p(topic, precision);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return (EXIT_SUCCESS);
}