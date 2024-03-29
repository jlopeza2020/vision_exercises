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

#include "image_geometry/pinhole_camera_model.h"


#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"


using namespace std::chrono_literals;
pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud);
geometry_msgs::msg::TransformStamped extrinsicbf2of; 

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
    
      // transform listener inialization
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Call on_timer function every 500ms
      timer_ = this->create_wall_timer(500ms, std::bind(&PCLSubscriber::on_timer, this));
      
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

    void on_timer(){

      try {
        // goes from base_footprint to optical frame 
        extrinsicbf2of = tf_buffer_->lookupTransform("head_front_camera_rgb_optical_frame", "base_footprint", tf2::TimePointZero);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
        return;
      }

    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_3d_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_3d_;
};

/**
  TO-DO
*/

//function from library did not work
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

//function from library did not work
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

  // Convert from RGB to HSV
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
  PointCloudXYZRGB2XYZHSV(cloud_in, *cloud_hsv);

  
  for(size_t i = 0; i < cloud_hsv->size(); i++){
    float h = cloud_hsv->points[i].h*(255.0/360.0);
    float s = cloud_hsv->points[i].s*255.0;
    float v = cloud_hsv->points[i].v*255.0;
    
    if((h >= 200.0 && s >= 190.0 && v >= 0.0 && h <= 230.0 && s <= 255.0 && v<= 255.0 )){

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

  // Convert from HSV to RGB
  PointCloudXYZHSV2XYZRGB(cloud_hsv_filtered, cloud_out);

  return cloud_out;
}


pcl::PointCloud<pcl::PointXYZRGB> remove_outliers(pcl::PointCloud<pcl::PointXYZRGB> cloud){
 
  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud.makeShared());
  // Set the number of neighbours to calculate the std desviation
  sor.setMeanK(50); 
  //Set threshold to eliminate outliers
  sor.setStddevMulThresh(1.0);
  sor.filter(cloud_filtered);

  return cloud_filtered;
}

void print_cubes(pcl::PointCloud<pcl::PointXYZRGB>& cloud, float x_center, float y_center,float z_center, int r, int g, int b){

  float dim = 0.15;
  float step = 0.008;
  
  for(float i = 0.0; i < dim; i+= step){
    for(float j = 0.0; j < dim; j+= step){
      for(float k = 0.0; k < dim; k+= step){

        pcl::PointXYZRGB point;
        point.x = x_center + i;
        point.y = y_center + j;
        point.z = z_center + k;
        point.r = r;
        point.g = g;
        point.b = b;
        cloud.push_back(point);

      }
    }
  }
}

void detect_spheres(pcl::PointCloud<pcl::PointXYZRGB>& in_cloud)
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

  // While the original cloud is still there
  while (cloud_filtered->size () > 0.0)
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

    print_cubes(in_cloud, x_center, y_center,z_center, 0, 0, 255);

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
  }
}

void lines_from_3D_to_2D(pcl::PointCloud<pcl::PointXYZRGB>& cloud){

  
  int r = 255; 
  int g = 0; 
  int b = 0;

  for(int i = 3; i <= 8; i++){

    //Left point
    geometry_msgs::msg::PointStamped point_in_left;
    point_in_left.header.frame_id = "base_footprint";
    point_in_left.point.x = i;
    point_in_left.point.y = 1.4;
    point_in_left.point.z = 0;

    geometry_msgs::msg::PointStamped point_out_left;
    point_out_left.header.frame_id = "head_front_camera_rgb_optical_frame";

    // Transform the point from base footprint to optical frame
    tf2::doTransform(point_in_left, point_out_left, extrinsicbf2of);

    print_cubes(cloud,point_out_left.point.x, point_out_left.point.y, point_out_left.point.z, r, g, b);

    //Right point
    geometry_msgs::msg::PointStamped point_in_right;
    point_in_right.header.frame_id = "base_footprint";
    point_in_right.point.x = i;
    point_in_right.point.y = -1.4;
    point_in_right.point.z = 0;

    geometry_msgs::msg::PointStamped point_out_right;
    point_out_right.header.frame_id = "head_front_camera_rgb_optical_frame";

    // Transform the point from base footprint to optical frame
    tf2::doTransform(point_in_right, point_out_right, extrinsicbf2of);

    print_cubes(cloud, point_out_right.point.x, point_out_right.point.y, point_out_right.point.z, r, g, b);
    r -= 40;
    g += 40;
    b += 40;

  }
}

pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
{
  // Create pointclouds
  pcl::PointCloud<pcl::PointXYZRGB> outlier_pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> inlier_pointcloud;

  outlier_pointcloud = get_hsv(in_pointcloud);
  inlier_pointcloud = remove_outliers(outlier_pointcloud);
  detect_spheres(inlier_pointcloud);
  lines_from_3D_to_2D(inlier_pointcloud);

  return inlier_pointcloud;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLSubscriber>());
  rclcpp::shutdown();
  return 0;
}