/*
# Copyright (c) 2022 Julia López Augusto
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <image_transport/image_transport.hpp>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <vector>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


int key;
bool print_once = true;
pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud);

class PCLSubscriber : public rclcpp::Node
{
  public:
    PCLSubscriber()
    : Node("opencv_subscriber")
    {
      auto qos = rclcpp::QoS( rclcpp::QoSInitialization( RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5 ));
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

      subscription_3d_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/head_front_camera/depth_registered/points", qos, std::bind(&PCLSubscriber::topic_callback_3d, this, std::placeholders::_1));
    
      publisher_3d_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "pcl_points", qos);
    }

  private:

    void topic_callback_3d(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {    
      
      // Convert to PCL data type
      pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
      pcl::fromROSMsg(*msg, point_cloud);     

      // PCL Processing
      pcl::PointCloud<pcl::PointXYZRGB> pcl_pointcloud = pcl_processing(point_cloud);
      
      // Convert to ROS data type
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(pcl_pointcloud, output);
      output.header = msg->header;

      // Publish the data
      publisher_3d_ -> publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_3d_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_3d_;
};

/**
  TO-DO
*/

void PointCloudXYZRGB2XYZHSV(pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZHSV>& out)
{
  out.width = in.width;
  out.height = in.height;
  for (const auto &point : in){
    pcl::PointXYZHSV p;
    pcl::PointXYZRGBtoXYZHSV(point, p);
    out.push_back(p);
  }
}

void PointCloudXYZHSV2XYZRGB(pcl::PointCloud<pcl::PointXYZHSV>& in, pcl::PointCloud<pcl::PointXYZRGB>& out)
{
  out.width = in.width;
  out.height = in.height;
  for (const auto &point : in){
    pcl::PointXYZRGB p;
    pcl::PointXYZHSVtoXYZRGB(point, p);
    out.push_back(p);
  }
}
 
pcl::PointCloud<pcl::PointXYZRGB> get_hsv(pcl::PointCloud<pcl::PointXYZRGB> cloud_in){
//pcl::PointCloud<pcl::PointXYZHSV> get_hsv(pcl::PointCloud<pcl::PointXYZRGB> cloud){

  pcl::PointCloud<pcl::PointXYZRGB> cloud_out;

  // Convertir de RGB a HSV
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
  //PointXYZRGB2XYZHSV(cloud_in, *cloud_hsv);
  PointCloudXYZRGB2XYZHSV(cloud_in, *cloud_hsv);

  // Crear una condición para filtrar los puntos de color rosa
  pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZHSV>);
  color_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
                                 new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::GE, 297)));
  color_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
                                 new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::LE, 300)));
  color_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
                                 new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::GE, 254/255)));
  color_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
                                 new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::LE, 255/255)));
  color_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
                                 new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::GE, 254/255)));
  color_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
                                 new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::LE, 255/255)));

  // Aplicar el filtro de condición para filtrar los puntos de color rosa
  pcl::ConditionalRemoval<pcl::PointXYZHSV> condrem;
  condrem.setCondition(color_cond);
  condrem.setInputCloud(cloud_hsv);
  condrem.setKeepOrganized(true);
  condrem.filter(*cloud_hsv);

  // Convertir de vuelta de HSV a RGB
  PointCloudXYZHSV2XYZRGB(*cloud_hsv, cloud_out);

  return cloud_out;
}


pcl::PointCloud<pcl::PointXYZRGB> remove_outliers(pcl::PointCloud<pcl::PointXYZRGB> cloud){
  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud.makeShared());
  sor.setMeanK(50); // Establecer el número de vecinos a considerar para el cálculo de la desviación estándar
  sor.setStddevMulThresh(1.0); // Establecer el umbral para eliminar los outliers
  sor.filter(cloud_filtered);

  return cloud_filtered;
}

void detect_spheres(pcl::PointCloud<pcl::PointXYZRGB> in_cloud){

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> center_cubes;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(in_cloud));

  //Create model of sphere

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setMaxIterations(1000);

  //while(cloud->points.size() > 0 ){
    //Adjust model
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  //if(inliers->indices.size() < 100){
  //  break;
  //}
  std::cout << "coef esfera" << std::endl;
  std::cout << "Centro x" << coefficients->values[0] << std::endl;
  std::cout << "Centro y" << coefficients->values[1] << std::endl;
  std::cout << "Centro z" << coefficients->values[2] << std::endl;
  std::cout << "Radio:" << coefficients->values[3] << std::endl;
  //}

}

void ejemplo (pcl::PointCloud<pcl::PointXYZRGB> in_cloud)
{
  //pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>(in_cloud));

  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Fill in the cloud data
  //pcl::PCDReader reader;
  //reader.read ("table_scene_lms400.pcd", *cloud_blob);

  //std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  //sor.setInputCloud (cloud_blob);
  //sor.setLeafSize (0.01f, 0.01f, 0.01f);
  //sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  //pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  //std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  //pcl::PCDWriter writer;
  //writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_SPHERE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  //pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointXYZRGB extract;


  int i = 0, nr_points = (int) cloud_filtered->size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    //std::stringstream ss;
    //ss << "table_scene_lms400_plane_" << i << ".pcd";
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }
}




//modify point cloud
pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
{
  // Create output pointcloud
  //pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> outlier_pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> inlier_pointcloud;


  // Processing
  //out_pointcloud = in_pointcloud;

  outlier_pointcloud = get_hsv(in_pointcloud);
  inlier_pointcloud = remove_outliers(outlier_pointcloud);
  //detect_spheres(inlier_pointcloud);
  ejemplo(inlier_pointcloud);

  

  return inlier_pointcloud;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLSubscriber>());
  rclcpp::shutdown();
  return 0;
}