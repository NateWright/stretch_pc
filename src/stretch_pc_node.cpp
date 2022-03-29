#include <iostream>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/common/distances.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  ros::Publisher pointCloudPub;
  ros::Publisher pointPub;

  // Indicies and Main Cloud
  std::vector<pcl::PointIndices> clusters;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  // Frames for publishing
  std::string targetFrame;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener *tfListener;
public:
  pc(std::string pointCloudTopic, float precision){
    this->pointCloudTopic = pointCloudTopic;
    this->precision = precision;

    tfListener = new tf2_ros::TransformListener(tfBuffer);
    sourcePointcloud = node.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &pc::segmentation, this);
    pointClicked = node.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &pc::pointPickingEventOccurred, this);

    pointCloudPub = node.advertise<sensor_msgs::PointCloud2>("/stretch_pc/pointcloud", 1000);
    pointPub = node.advertise<geometry_msgs::PointStamped>("/stretch_pc/centerPoint", 1000);;
  }
  ~pc(){
    delete tfListener;
  }

  void segmentation(sensor_msgs::PointCloud2 input) {
    targetFrame = input.header.frame_id;

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

    sensor_msgs::PointCloud2 p;
    pcl::toROSMsg(*colored_cloud.get(), p);
    p.header = input.header;
    pointCloudPub.publish(p);

    return;
  }
  void pointPickingEventOccurred (geometry_msgs::PointStamped inputPoint) {
    sourcePointcloud.shutdown();
    
    geometry_msgs::PointStamped tfPoint = tfBuffer.transform(inputPoint, targetFrame);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);

    float x, y, z;
    x = tfPoint.point.x;
    y = tfPoint.point.y;
    z = tfPoint.point.z;

    for (pcl::PointIndices p : clusters)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (const auto& idx : p.indices){
        cloud_cluster->push_back ((*colored_cloud)[idx]);
        if(pcl::euclideanDistance(pcl::PointXYZ(x,y,z), pcl::PointXYZ(cloud_cluster->back().x, cloud_cluster->back().y, cloud_cluster->back().z)) < precision){
          output = cloud_cluster;
        }
      }
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
    }

    x = 0; y = 0; z = 0;
    int count = 0;
    for(auto p : output->points){
      x += p.x;
      y += p.y;
      z += p.z;
      count++;
    }

    geometry_msgs::PointStamped pStamped;
    pStamped.point.x = x/count;
    pStamped.point.y = y/count;
    pStamped.point.z = z/count;
    pStamped.header = tfPoint.header;
    pointPub.publish(pStamped);

    sensor_msgs::PointCloud2 p;
    pcl::toROSMsg(*output.get(), p);
    p.header = tfPoint.header;
    pointCloudPub.publish(p);
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_translation");
  ros::NodeHandle n;

  std::string topic;
  float precision;
  if(!n.getParam("/stretch_pc/pointCloudTopic", topic)){
    ROS_DEBUG("stretch_pc: No pointcloud topic found\n");
    return (EXIT_FAILURE);
  }
  if(!n.getParam("/stretch_pc/precision", precision)){
    ROS_DEBUG("stretch_pc: No precision found\n");
    return (EXIT_FAILURE);
  }

  pc p(topic, precision);
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }

  return (EXIT_SUCCESS);
}