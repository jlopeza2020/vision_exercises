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

  pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
  pcl::PointCloud<pcl::PointXYZHSV> cloud_hsv_filtered;

  // Convertir de RGB a HSV
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
  PointCloudXYZRGB2XYZHSV(cloud_in, *cloud_hsv);

  
  for(size_t i = 0; i < cloud_hsv->size(); ++i){
    float h = cloud_hsv->points[i].h*(255.0/360.0);
    float s = cloud_hsv->points[i].s*255.0;
    float v = cloud_hsv->points[i].v*255.0;
    
    if((h >= 200.0 && s >= 190.0 && v >= 0.0 && h <= 220.0 && s <= 255.0 && v<= 255.0 )){

      pcl::PointXYZHSV point;
      point.x = cloud_hsv->points[i].x;
      point.y = cloud_hsv->points[i].y;
      point.z = cloud_hsv->points[i].z;
      point.h = cloud_hsv->points[i].h;
      point.s = cloud_hsv->points[i].s;
      point.v = cloud_hsv->points[i].v;
      cloud_hsv_filtered.push_back(point);
    }
  }

  // Convertir de vuelta de HSV a RGB
  PointCloudXYZHSV2XYZRGB(cloud_hsv_filtered, cloud_out);

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


void draw_square(pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered , float x_center, float y_center, float z_center)
{

  // Definir las dimensiones del cubo
  float cube_size = 0.3;

  // Crear las 8 esquinas del cubo
  pcl::PointXYZRGB v0, v1, v2, v3, v4, v5, v6, v7;

  v0.x = x_center - cube_size/2; v0.y = y_center - cube_size/2; v0.z = z_center - cube_size/2;
  v1.x = x_center + cube_size/2; v1.y = y_center - cube_size/2; v1.z = z_center - cube_size/2;
  v2.x = x_center + cube_size/2; v2.y = y_center + cube_size/2; v2.z = z_center - cube_size/2;
  v3.x = x_center - cube_size/2; v3.y = y_center + cube_size/2; v3.z = z_center - cube_size/2;
  v4.x = x_center - cube_size/2; v4.y = y_center - cube_size/2; v4.z = z_center + cube_size/2;
  v5.x = x_center + cube_size/2; v5.y = y_center - cube_size/2; v5.z = z_center + cube_size/2;
  v6.x = x_center + cube_size/2; v6.y = y_center + cube_size/2; v6.z = z_center + cube_size/2;
  v7.x = x_center - cube_size/2; v7.y = y_center + cube_size/2; v7.z = z_center + cube_size/2;

  // Establecer el color del cubo en rojo (R=255, G=0, B=0)
  v0.r = 0; v0.g = 0; v0.b = 255;
  v1.r = 0; v1.g = 0; v1.b = 255;
  v2.r = 0; v2.g = 0; v2.b = 255;
  v3.r = 0; v3.g = 0; v3.b = 255;
  v4.r = 0; v4.g = 0; v4.b = 255;
  v5.r = 0; v5.g = 0; v5.b = 255;
  v6.r = 0; v6.g = 0; v6.b = 255;
  v7.r = 0; v7.g = 0; v7.b = 255;

  // Agregar los vértices del cubo al PointCloud
  cloud_filtered.push_back(v0);
  cloud_filtered.push_back(v1);
  cloud_filtered.push_back(v2);
  cloud_filtered.push_back(v3);
  cloud_filtered.push_back(v4);
  cloud_filtered.push_back(v5);
  cloud_filtered.push_back(v6);
  cloud_filtered.push_back(v7);

  
}

/*void detect_spheres(pcl::PointCloud<pcl::PointXYZRGB> in_cloud){

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> center_cubes;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(in_cloud));

  //Create model of sphere

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setMaxIterations(1000);

  while(cloud->points.size() > 0 ){
    //Adjust model
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size() < 100){
      break;
    }
    std::cout << "coef esfera" << std::endl;
    std::cout << "Centro x" << coefficients->values[0] << std::endl;
    std::cout << "Centro y" << coefficients->values[1] << std::endl;
    std::cout << "Centro z" << coefficients->values[2] << std::endl;
    std::cout << "Radio:" << coefficients->values[3] << std::endl;

    draw_square(cloud, coefficients->values[0], coefficients->values[1], coefficients->values[2]);

  }


}*/

void detect_spheres(pcl::PointCloud<pcl::PointXYZRGB> in_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>(in_cloud));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  
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

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  int i = 0, nr_points = (int) cloud_filtered->size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->size () > 0.3 * nr_points)
  {
    // Segment the largest sphere component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a sphere model for the given dataset." << std::endl;
      break;
    }

    float x_center = coefficients->values[0];
    float y_center = coefficients->values[1];
    float z_center = coefficients->values[2];

    float max= 0.15;
    float advance = 0.008;

    for(float i = 0.0; i < max; i+= advance){
      for(float j = 0.0; j < max; j+= advance){
        for(float k = 0.0; k < max; k+= advance){

          pcl::PointXYZRGB point;
          point.x = x_center + i;
          point.y = y_center + j;
          point.z = z_center + k;
          point.r = 0;
          point.g = 0;
          point.b = 255;
          in_cloud.push_back(point);

        }

      }

    }

    //draw_square(in_cloud, coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

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
  // Create pointclouds
  //pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> outlier_pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> inlier_pointcloud;

  outlier_pointcloud = get_hsv(in_pointcloud);
  inlier_pointcloud = remove_outliers(outlier_pointcloud);
  detect_spheres(inlier_pointcloud);

  return inlier_pointcloud;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLSubscriber>());
  rclcpp::shutdown();
  return 0;
}