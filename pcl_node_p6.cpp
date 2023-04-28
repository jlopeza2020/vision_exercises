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

int key;
bool print_once = true;
pcl::PointCloud<pcl::PointXYZHSV> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud);

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
      pcl::PointCloud<pcl::PointXYZHSV> pcl_pointcloud = pcl_processing(point_cloud);
      
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

//pcl::PointCloud<pcl::PointXYZRGB> get_hsv( pcl::PointCloud<pcl::PointXYZRGB> cloud, int min_h, int min_s ,int min_v, int max_h, int max_s, int max_v){
pcl::PointCloud<pcl::PointXYZHSV> get_hsv(pcl::PointCloud<pcl::PointXYZRGB> cloud){

  //pcl::PointCloud<pcl::PointXYZHSV> cloud_hsv_nptr;

  // convert image in hsv 

  //pcl::PointXYZRGBtoXYZHSV(in_plc, plc_HSV);
  //cv::cvtColor(in_image, img_inHSV, cv::COLOR_BGR2HSV);
  // Detect the object in green
  //cv::inRange(img_inHSV, cv::Scalar(min_h, min_s, min_v), cv::Scalar(max_h,max_s,max_v), green_dt);
  // Edge detection
  //Canny(green_dt, out_img, 50, 200, 3);
  //std::vector<cv::Vec2f> lines;   // will hold the results of the detection (rho, theta)
  //HoughLines(out_img, lines, 1, CV_PI / 180, value_hough, 0, 0);   // runs the actual detection




  // Cargamos la nube de puntos RGB
  //PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);


  // Convertimos la nube de puntos RGB a HSV
  cv::Mat img_hsv(cloud.height, cloud.width, CV_8UC3);
  for (size_t i = 0; i < cloud.size(); i++) {
    cv::Vec3b rgb(cloud.points[i].r, cloud.points[i].g, cloud.points[i].b);
    cv::Vec3b hsv;
    cv::cvtColor(cv::Mat(rgb), cv::Mat(hsv), cv::COLOR_BGR2HSV);
    img_hsv.at<cv::Vec3b>(i / cloud.width, i % cloud.width) = hsv;
  }

  // Convertimos la imagen HSV a una nube de puntos
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
  for (int v = 0; v < img_hsv.rows; v++) {
    for (int u = 0; u < img_hsv.cols; u++) {
      cv::Vec3b hsv = img_hsv.at<cv::Vec3b>(v, u);
      pcl::PointXYZHSV point;
      point.x = cloud.at(u, v).x;
      point.y = cloud.at(u, v).y;
      point.z = cloud.at(u, v).z;
      point.h = hsv[0];
      point.s = hsv[1];
      point.v = hsv[2];
      cloud_hsv->push_back(point);
    }
  }

  // Convertir de puntero a objeto
  pcl::PointCloud<pcl::PointXYZHSV> cloud_hsv_nptr = *cloud_hsv;

  return cloud_hsv_nptr;
}


//modify point cloud
pcl::PointCloud<pcl::PointXYZHSV> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
{
  // Create output pointcloud
  pcl::PointCloud<pcl::PointXYZHSV> out_pointcloud;

  // Processing
  //out_pointcloud = in_pointcloud;

  /*cv::Mat test = cv::Mat::zeros(cv::Size(600,100), CV_32FC1);

  int max_h = 255;
  int max_s =  255;
  int max_v =  255;
  int min_h = 255;
  int min_s =  255;
  int min_v =  255;
  int zero = 0;


  key = cv::pollKey();

  if(print_once){
    cv::namedWindow("P5");

    cv::createTrackbar("max H", "P5", nullptr, max_h, 0);
    cv::setTrackbarPos("max H", "P5", zero);
    cv::createTrackbar("max S", "P5", nullptr, max_s, 0);
    cv::setTrackbarPos("max S", "P5", zero);
    cv::createTrackbar("max V", "P5", nullptr, max_v, 0);
    cv::setTrackbarPos("max V", "P5", zero);
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

  out_pointcloud = get_hsv(in_pointcloud);

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