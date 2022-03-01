#include <iostream>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/common/distances.h>

using namespace std::chrono_literals;

class pc
{
private:
  ros::NodeHandle node;
  ros::Subscriber input;
  ros::Publisher chatter_pub;
  ros::Subscriber pointClicked;

  std::vector<pcl::PointIndices> clusters;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
public:
  pc(){
    input = node.subscribe<sensor_msgs::PointCloud2>("/realsense/depth/color/points", 1, &pc::segmentation, this);
    pointClicked = node.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &pc::pointPickingEventOccurred, this);

    chatter_pub = node.advertise<sensor_msgs::PointCloud2>("chatter", 1000);
  }
  void segmentation(sensor_msgs::PointCloud2 input){
    ROS_INFO("%s", input.header.frame_id.c_str());
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);


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
    // ROS_INFO("segmentation run");

    sensor_msgs::PointCloud2 p;
    pcl::toROSMsg(*colored_cloud.get(), p);
    // ROS_INFO("done toROSMsg\n");
    p.header = input.header;
    chatter_pub.publish(p);
    // ROS_INFO("done");

    return;
  }
  void pointPickingEventOccurred (geometry_msgs::PointStamped inputPoint)
  {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    https://answers.ros.org/question/215485/how-to-transform-a-pointstamped-from-a-frame-to-another/
    tfBuffer.transform("camera_depth_optical_frame", )
    geometry_msgs::Point p(inputPoint.point);
    geometry_msgs::Point point = tfBuffer.transform<geometry_msgs::Point>(p, "camera_depth_optical_frame");
    ROS_INFO("%s", point.header.frame_id.c_str());
    input.shutdown();
    std::cout << "[INFO] Point picking event occurred." << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);

    float x, y, z;
    x = point.point.x;
    y = point.point.y;
    z = point.point.z;

    for (pcl::PointIndices p : clusters)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (const auto& idx : p.indices){
        cloud_cluster->push_back ((*colored_cloud)[idx]);
        // ROS_INFO("%f",pcl::euclideanDistance(pcl::PointXYZ(x,y,z), pcl::PointXYZ(cloud_cluster->back().x, cloud_cluster->back().y, cloud_cluster->back().z)));
        if(pcl::euclideanDistance(pcl::PointXYZ(x,y,z), pcl::PointXYZ(cloud_cluster->back().x, cloud_cluster->back().y, cloud_cluster->back().z)) < 2){
          output = cloud_cluster;
        }
      }
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
    }
    sensor_msgs::PointCloud2 p;
    pcl::toROSMsg(*output.get(), p);
    p.header = point.header;
    chatter_pub.publish(p);
    ROS_INFO("done cloud update");
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_translation");
  ros::NodeHandlePtr n;

  pc p;
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }

  return (EXIT_SUCCESS);
}