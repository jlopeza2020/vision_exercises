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
 

/*pcl::PointCloud<pcl::PointXYZRGB> get_hsv(pcl::PointCloud<pcl::PointXYZRGB> cloud_in, int h_min, int s_min,int v_min, int h_max, int s_max, int v_max){
//pcl::PointCloud<pcl::PointXYZHSV> get_hsv(pcl::PointCloud<pcl::PointXYZRGB> cloud){

  pcl::PointCloud<pcl::PointXYZRGB> cloud_out;

  // Convertir de RGB a HSV
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
  //PointXYZRGB2XYZHSV(cloud_in, *cloud_hsv);
  PointCloudXYZRGB2XYZHSV(cloud_in, *cloud_hsv);

  // Crear una condición para filtrar los puntos de color rosa
  pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZHSV>);
  color_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
                                 new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::GE, h_min)));
  color_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
                                 new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::LE, h_max)));
  color_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
                                 new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::GE, s_min/255)));
  color_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
                                 new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::LE, s_max/255)));
  color_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
                                 new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::GE, v_min/255)));
  color_cond->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::ConstPtr(
                                 new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::LE, v_max/255)));

  // Aplicar el filtro de condición para filtrar los puntos de color rosa
  pcl::ConditionalRemoval<pcl::PointXYZHSV> condrem;
  condrem.setCondition(color_cond);
  condrem.setInputCloud(cloud_hsv);
  condrem.setKeepOrganized(true);
  condrem.filter(*cloud_hsv);

  // Convertir de vuelta de HSV a RGB
  PointCloudXYZHSV2XYZRGB(*cloud_hsv, cloud_out);

  return cloud_out;
}*/

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


/*pcl::PointCloud<pcl::PointXYZRGB> get_inlier(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
  ///pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Fill in the cloud data
  //pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  //reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

  //std::cerr << "Cloud before filtering: " << std::endl;
  //std::cerr << *cloud << std::endl;

  // Create the filtering object
  //pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  //std::cerr << "Cloud after filtering: " << std::endl;
  //std::cerr << *cloud_filtered << std::endl;

  //pcl::PCDWriter writer;
  //writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

  ///sor.setNegative (true);
  //sor.filter (*cloud_filtered);
  //writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

  return *cloud_filtered;
}*/

pcl::PointCloud<pcl::PointXYZRGB> removeColorOutliers(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud.makeShared());
  //sor.setInputColors(cloud.makeShared());
  sor.setMeanK(50); // Establecer el número de vecinos a considerar para el cálculo de la desviación estándar
  sor.setStddevMulThresh(1.0); // Establecer el umbral para eliminar los outliers
  //sor.setColorThreshold(50); // Establecer el umbral para eliminar los outliers de color
  sor.filter(cloud_filtered);

  return cloud_filtered;
}


//modify point cloud
pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
{
  // Create output pointcloud
  pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> outlier_pointcloud;

  // Processing
  //out_pointcloud = in_pointcloud;

  /*cv::Mat test = cv::Mat::zeros(cv::Size(600,100), CV_32FC1);

  int max_h = 360;
  int max_s =  255;
  int max_v =  255;
  int min_h = 360;
  int min_s =  255;
  int min_v =  255;
  int zero = 0;


  key = cv::pollKey();

  if(print_once){
    cv::namedWindow("P5");

    cv::createTrackbar("max H", "P5", nullptr, max_h, 0);
    cv::setTrackbarPos("max H", "P5", max_h);
    cv::createTrackbar("max S", "P5", nullptr, max_s, 0);
    cv::setTrackbarPos("max S", "P5", max_s);
    cv::createTrackbar("max V", "P5", nullptr, max_v, 0);
    cv::setTrackbarPos("max V", "P5", max_v);
    cv::createTrackbar("min H", "P5", nullptr, min_h, 0);
    cv::setTrackbarPos("min H", "P5", zero);
    cv::createTrackbar("min S", "P5", nullptr, min_s, 0);
    cv::setTrackbarPos("min S", "P5", zero);
    cv::createTrackbar("min V", "P5", nullptr, min_v, 0);
    cv::setTrackbarPos("min V", "P5", zero);
    print_once = false;
  }

  int gt_max_h = cv::getTrackbarPos("max H", "P5");
  int gt_max_s = cv::getTrackbarPos("max S", "P5");
  int gt_max_v = cv::getTrackbarPos("max V", "P5");
  int gt_min_h = cv::getTrackbarPos("min H", "P5");
  int gt_min_s = cv::getTrackbarPos("min S", "P5");
  int gt_min_v = cv::getTrackbarPos("min V", "P5");*/



  //out_pointcloud = get_hsv(in_pointcloud, gt_min_h, gt_min_s ,gt_min_v, gt_max_h, gt_max_s, gt_max_v);
  outlier_pointcloud = get_hsv(in_pointcloud);
  out_pointcloud = removeColorOutliers(outlier_pointcloud);


  //cv::imshow("P5",test);


  return out_pointcloud;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLSubscriber>());
  rclcpp::shutdown();
  return 0;
}