#include <ros/ros.h>

#include "pc.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_segmentation");
    ros::NodeHandlePtr n(new ros::NodeHandle());

    std::string topic;
    float precision;
    if (!n->getParam("/stretch_pc/pointCloudTopic", topic)) {
        ROS_DEBUG("stretch_pc: No pointcloud topic found\n");
        return (EXIT_FAILURE);
    }

    pc p(n, topic);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return (EXIT_SUCCESS);
}