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

using namespace std::chrono_literals;

std::vector<pcl::PointIndices> clusters;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
pcl::visualization::PCLVisualizer viewer("Cluster viewer");

pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentation(sensor_msgs::PointCloud2 input){
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

  return colored_cloud;
}

void pointPickingEventOccurred (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
  std::cout << "[INOF] Point picking event occurred." << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);

  float x, y, z;
  if (event.getPointIndex () == -1)
  {
    return;
  }
  event.getPoint(x, y, z);

  for (pcl::PointIndices p : clusters)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& idx : p.indices){
      cloud_cluster->push_back ((*colored_cloud)[idx]);
      if(cloud_cluster->back().x == x && cloud_cluster->back().y == y && cloud_cluster->back().z == z){
        output = cloud_cluster;
      }
    }
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
  }
  viewer.removeAllPointClouds();
  viewer.addPointCloud(output, "cloud");
}

int main()
{
  pcl::PCLPointCloud2 pcl_pc2;
  sensor_msgs::PointCloud2 input;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("1.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
  pcl::toPCLPointCloud2(*cloud, pcl_pc2);
  pcl_conversions::fromPCL(pcl_pc2, input);

  

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = segmentation(input);
  viewer.addPointCloud(colored_cloud, "cloud");
  viewer.registerPointPickingCallback (pointPickingEventOccurred, (void*)&viewer);
  viewer.spin();

  return (EXIT_SUCCESS);
}