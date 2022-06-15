#include <pcl/common/distances.h>
// #include <pcl/conversions.h>
#include <pcl/filters/filter_indices.h>  // for pcl::removeNaNFromPointCloud
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>
#include <thread>
#include <vector>

#pragma once

using namespace std::chrono_literals;

class pc {
   private:
    // Setup
    ros::NodeHandle node;
    std::string pointCloudTopic;
    float precision;

    // Publishers and Subscribers
    ros::Subscriber sourcePointcloud;
    ros::Subscriber pointClicked;
    ros::Subscriber resetSub;
    ros::Publisher pointCloudPub;
    ros::Publisher clusterPub;
    ros::Publisher pointPub;

    // Indicies and Main Cloud
    std::vector<pcl::PointIndices> clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

    // Frames for publishing
    std::string targetFrame;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;

   public:
    explicit pc(std::string pointCloudTopic, float precision) {
        this->pointCloudTopic = pointCloudTopic;
        this->precision = precision;

        tfListener = new tf2_ros::TransformListener(tfBuffer);
        sourcePointcloud = node.subscribe(pointCloudTopic, 1, &pc::segmentation, this);
        ROS_INFO_STREAM("Point cloud topic: " << pointCloudTopic);
        pointClicked = node.subscribe("/clicked_point", 1, &pc::pointPickingEventOccurred, this);

        pointCloudPub = node.advertise<sensor_msgs::PointCloud2>("/stretch_pc/pointcloud", 1000);
        clusterPub = node.advertise<sensor_msgs::PointCloud2>("/stretch_pc/cluster", 1000);
        pointPub = node.advertise<geometry_msgs::PointStamped>("/stretch_pc/centerPoint", 1000);
    }
    ~pc() { delete tfListener; }
    void segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc);
    void pointPickingEventOccurred(const geometry_msgs::PointStamped::ConstPtr);
};

void pc::segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc) {
    targetFrame = pc->header.frame_id;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pc;

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud);
    reg.setIndices(indices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(10);
    reg.setPointColorThreshold(6);
    reg.setRegionColorThreshold(5);
    reg.setMinClusterSize(600);

    reg.extract(clusters);

    colored_cloud = reg.getColoredCloud();

    colored_cloud->header.frame_id = targetFrame;

    pointCloudPub.publish(colored_cloud);

    return;
}

void pc::pointPickingEventOccurred(const geometry_msgs::PointStamped::ConstPtr inputPoint) {
    ROS_INFO_STREAM("Picking event occurred");
    sourcePointcloud.shutdown();

    geometry_msgs::PointStamped tfPoint = tfBuffer.transform(*inputPoint, targetFrame);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointXYZRGB p;
    p.x = tfPoint.point.x;
    p.y = tfPoint.point.y;
    p.z = tfPoint.point.z;

    ROS_INFO_STREAM("Looking for cluster");
    bool done = false;

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(colored_cloud);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    kdtree.nearestKSearch(p, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    int pos = pointIdxNKNSearch[0];

    for (pcl::PointIndices p : clusters) {
        cloud_cluster.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& idx : p.indices) {
            cloud_cluster->push_back((*colored_cloud)[idx]);
            if (idx == pos) {
                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                done = true;
            }
        }
        if (done) {
            ROS_INFO_STREAM("Cluster found");
            break;
        }
    }

    float x = 0,
          y = 0,
          z = 0;
    int count = 0;
    for (auto p : cloud_cluster->points) {
        x += p.x;
        y += p.y;
        z += p.z;
        count++;
    }

    geometry_msgs::PointStamped pStamped;
    pStamped.point.x = x / count;
    pStamped.point.y = y / count;
    pStamped.point.z = z / count;
    pStamped.header = tfPoint.header;
    pointPub.publish(pStamped);

    cloud_cluster->header.frame_id = colored_cloud->header.frame_id;
    clusterPub.publish(cloud_cluster);

    sourcePointcloud = node.subscribe(pointCloudTopic, 1, &pc::segmentation, this);
}