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
//cv::Matx33f K; //intrinsic values 
geometry_msgs::msg::TransformStamped extrinsicbf2of; 
cv::Matx34f extrinsic_matrixbf2of;

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

      //subscription_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      //"/head_front_camera/rgb/camera_info", qos, std::bind(&PCLSubscriber::topic_callback_in_params, this, std::placeholders::_1));
    
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

    /*void topic_callback_in_params(const sensor_msgs::msg::CameraInfo::SharedPtr msg) const
    {           
      //Create camera model
      image_geometry::PinholeCameraModel camera_model = image_geometry::PinholeCameraModel();
      camera_model.fromCameraInfo(msg);

      //Obtain intrinsic matrix
      K = camera_model.intrinsicMatrix();

    }*/

    void on_timer(){

      try {
        // goes from base_footprint to optical frame 
        extrinsicbf2of = tf_buffer_->lookupTransform("head_front_camera_rgb_optical_frame", "base_footprint", tf2::TimePointZero);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
        return;
      }

    }

    //rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_info_;
    //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

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

  // Convertir de RGB a HSV
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
  PointCloudXYZRGB2XYZHSV(cloud_in, *cloud_hsv);

  
  for(size_t i = 0; i < cloud_hsv->size(); ++i){
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

// need to be reevised
void print_cubes(pcl::PointCloud<pcl::PointXYZRGB>& cloud, float x_center, float y_center,float z_center){

  
  //pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
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
        cloud.push_back(point);

      }
    }
  }

  //pcl::PointCloud<pcl::PointXYZRGB> merged_cloud;
  //merged_cloud = cloud;
  //cloud += new_cloud;

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

  //int in = 0, nr_points = (int) cloud_filtered->size ();
  int in = 0;

  // While 30% of the original cloud is still there
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

    //std::cout <<"coeffx" << x_center << std::endl;
    //std::cout <<"cpeffy " <<  y_center << std::endl;
    // std::cout <<"cpeffz " <<  z_center << std::endl;

    print_cubes(in_cloud, x_center, y_center,z_center);

    /*float max= 0.15;
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
          in_cloud.push_back(point);*/

   //     }

     // }

    //}

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    in++;
  }
}

void lines_from_3D_to_2D(pcl::PointCloud<pcl::PointXYZRGB>& cloud){

  
  for(int i = 3; i <= 8; i++){

    //cv::Mat point_req1 = (cv::Mat_<float>(4,1) << i, 1.4, 0.0, 1.0);
    //cv::Mat point_req2 = (cv::Mat_<float>(4,1) << i, -1.4, 0.0, 1.0);

    geometry_msgs::msg::PointStamped point_in_left;
    point_in_left.header.frame_id = "base_footprint";
    point_in_left.point.x = i;
    point_in_left.point.y = 1.4;
    point_in_left.point.z = 0;


    geometry_msgs::msg::PointStamped point_out_left;
    point_out_left.header.frame_id = "head_front_camera_rgb_optical_frame";

    // Transform the point from "footprint" to "optical" frame
    tf2::doTransform(point_in_left, point_out_left, extrinsicbf2of);

    print_cubes(cloud, point_out_left.point.x, point_out_left.point.y, point_out_left.point.z);


    geometry_msgs::msg::PointStamped point_in_right;
    point_in_right.header.frame_id = "base_footprint";
    point_in_right.point.x = i;
    point_in_right.point.y = -1.4;
    point_in_right.point.z = 0;


    geometry_msgs::msg::PointStamped point_out_right;
    point_out_right.header.frame_id = "head_front_camera_rgb_optical_frame";

    // Transform the point from "footprint" to "optical" frame
    tf2::doTransform(point_in_right, point_out_right, extrinsicbf2of);

    print_cubes(cloud, point_out_right.point.x, point_out_right.point.y, point_out_right.point.z);




    //return 0;

    //cv::Mat res = K*extrinsic_matrixbf2of*point_req1;
    //cv::Mat res2 = K*extrinsic_matrixbf2of*point_req2;

    

    //float x_center_left = res.at<float>(0, 0)/abs(res.at<float>(2, 0));
    //float y_center_left = res.at<float>(1, 0)/abs(res.at<float>(2, 0));
    //float z_center_left = res.at<float>(2, 0);

    //std::cout <<"x_left" << x_center_left/1000 << std::endl;
    //std::cout <<"y left" <<  y_center_left/1000 << std::endl;
    
    //print_cubes(cloud, x_center_left/1000, y_center_left/1000, z_center_left);//{

    //float x_center_right = res2.at<float>(0, 0)/abs(res2.at<float>(2, 0));
    //float y_center_right = res2.at<float>(1, 0)/abs(res2.at<float>(2, 0));
    //float z_center_right = res2.at<float>(2, 0);

    //std::cout <<"x_right" << x_center_right/1000 << std::endl;
    //std::cout <<"y right" << y_center_right/1000 << std::endl;

    //print_cubes(cloud, x_center_right/1000, y_center_right/1000, z_center_right);

    //cv::Point center(res.at<float>(0, 0)/abs(res.at<float>(2, 0)), res.at<float>(1, 0)/abs(res.at<float>(2, 0)));
    //cv::circle(out_image,center, 3, cv::Scalar(0, 0, 255), 2); // draw the circle on the image

    //cv::Point center2(res2.at<float>(0, 0)/abs(res2.at<float>(2, 0)),res2.at<float>(1, 0)/abs(res2.at<float>(2, 0)));
    //cv::circle(out_image,center2, 3, cv::Scalar(0, 0, 255), 2); // draw the circle on the image


  }
}

//modify point cloud
pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
{
  // Create pointclouds
  pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> outlier_pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> inlier_pointcloud;

  //auto rotation = extrinsicbf2of.transform.rotation;
  
  //tf2::Matrix3x3 mat(tf2::Quaternion{rotation.x, rotation.y, rotation.z, rotation.w});
  
  //extrinsic_matrixbf2of = cv::Matx34f( mat[0][0], mat[0][1], mat[0][2], extrinsicbf2of.transform.translation.x,
                                  //mat[1][0], mat[1][1], mat[1][2], extrinsicbf2of.transform.translation.y,
                                  //mat[2][0], mat[2][1], mat[2][2], extrinsicbf2of.transform.translation.z);
  

 //out_pointcloud = in_pointcloud;

  //fix it 
  //lines_from_3D_to_2D(in_pointcloud);

  outlier_pointcloud = get_hsv(in_pointcloud);
  inlier_pointcloud = remove_outliers(outlier_pointcloud);
  detect_spheres(inlier_pointcloud);
  lines_from_3D_to_2D(inlier_pointcloud);
  //lines_from_3D_to_2D(out_pointcloud);


  return inlier_pointcloud;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLSubscriber>());
  rclcpp::shutdown();
  return 0;
}