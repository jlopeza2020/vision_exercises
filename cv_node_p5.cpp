/*
# Copyright (c) 2023 Julia López Augusto
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
#include "image_geometry/pinhole_camera_model.h"


#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf2/LinearMath/Transform.h>
#include <Eigen/Dense>

using namespace std::chrono_literals;

int key;
bool print_once = true;
cv::Matx33f K; //intrinsic values 

geometry_msgs::msg::TransformStamped extrinsic; 

cv::Matx34f extrinsic_matrix;
cv::Mat res;
cv::Mat res2;

cv::Mat image_processing(const cv::Mat in_image);

class ComputerVisionSubscriber : public rclcpp::Node
{
  public:
    ComputerVisionSubscriber()
    : Node("opencv_subscriber")
    {
      auto qos = rclcpp::QoS( rclcpp::QoSInitialization( RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5 ));
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
   

      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/head_front_camera/rgb/image_raw", qos, std::bind(&ComputerVisionSubscriber::topic_callback, this, std::placeholders::_1));

      subscription_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/head_front_camera/rgb/camera_info", qos, std::bind(&ComputerVisionSubscriber::topic_callback_in_params, this, std::placeholders::_1));
    
    
      // Inicializar el transform listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Call on_timer function every 500ms
      timer_ = this->create_wall_timer(500ms, std::bind(&ComputerVisionSubscriber::on_timer, this));


      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("cv_image", qos);
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {     
      // Convert ROS Image to CV Image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat image_raw =  cv_ptr->image;

      // Image processing
      cv::Mat cv_image = image_processing(image_raw);

      // Convert OpenCV Image to ROS Image
      cv_bridge::CvImage img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, cv_image);
      sensor_msgs::msg::Image out_image; // >> message to be sent
      img_bridge.toImageMsg(out_image); // from cv_bridge to sensor_msgs::Image

      // Publish the data
      publisher_ -> publish(out_image);

    }

    void topic_callback_in_params(const sensor_msgs::msg::CameraInfo::SharedPtr msg) const
    {     
      // Convert ROS Image to CV Image

      // Crear modelo de cámara
      image_geometry::PinholeCameraModel camera_model = image_geometry::PinholeCameraModel();
      camera_model.fromCameraInfo(msg);

      // Obtener matriz de parámetros intrínsecos
      K = camera_model.intrinsicMatrix();

    }

    void on_timer(){

      try {
        extrinsic = tf_buffer_->lookupTransform("head_front_camera_rgb_optical_frame", "base_footprint", tf2::TimePointZero);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
        return;
      }

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_info_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

};

/**
  TO-DO COMPLETE THIS PART 
**/

// create mouse callback

std::vector<cv::Point> points;

void on_mouse(int event, int x, int y, int, void*)
{
  if (event == cv::EVENT_LBUTTONDOWN){
    points.push_back(cv::Point(x, y));
  }
      //std::cout << "Left button" << std::endl;
     //points.push_back(Point(x, y));
      //break;
    //case cv::EVENT_RBUTTONDOWN:
    //  std::cout << "Right button" << std::endl;
    //  break;
    //default:
    //  std::cout << "No button" << std::endl;
    //  break;
    //}
}

cv::Mat detect_skeleton(cv::Mat in_image, int iters){


  cv::Mat out_image, img_inHSV;

  cv::cvtColor(in_image, img_inHSV, cv::COLOR_BGR2HSV);
  // Detect the object in green
  cv::inRange(img_inHSV, cv::Scalar(0, 0, 109), cv::Scalar(255,255,117), out_image);

  cv::Mat in_clone = in_image.clone();


  // crear una imagen esqueleto vacía
  cv::Mat skeleton = cv::Mat::zeros(in_image.size(), CV_8UC1);

    // realizar la operación de esqueletización
  for (int i = 0; i < iters; i++) {
    // realizar una apertura de la imagen
    cv::Mat opened;
    morphologyEx(out_image, opened, cv::MORPH_OPEN, cv::Mat());

    // restar la imagen abierta de la imagen original
    cv::Mat temp = out_image - opened;

    // erosionar la imagen original
    erode(out_image, out_image, cv::Mat());

    // unir la imagen esqueleto y la imagen temp
    bitwise_or(skeleton, temp, skeleton);
  }

  for (int i = 0; i < in_clone.cols; i++) {
    for (int j = 0; j < in_clone.rows; j++) {
      cv::Scalar intensity = skeleton.at<uchar>(j, i);
      if (intensity.val[0] == 255) {
        in_clone.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 255, 0);
      }
    }
  }

  return in_clone;


}


cv::Mat image_processing(const cv::Mat in_image) 
{
  
  int max_value_choose_opt = 2;
  int init_value_choose_opt = 0;
  int max_value_iters = 100;
  int init_value_iters = 20;
  int max_value_distance = 8;
  int init_value_distance = 0;

  cv::Mat out_image;
  cv::Mat point_req1 = (cv::Mat_<float>(4,1) << 3.0, 1.4, 0.0, 1.0);

  cv::Mat point_req2 = (cv::Mat_<float>(4,1) << 3.0, -1.4, 0.0, 1.0);

  extrinsic_matrix = cv::Matx34f(   0, 1, 0, extrinsic.transform.translation.x,
                                    0, 0, 1, extrinsic.transform.translation.y,
                                    1, 0, 0, extrinsic.transform.translation.z);

                
                                  
  res = K*extrinsic_matrix*point_req1;
  res2 = K*extrinsic_matrix*point_req2;

  std::cout << K << std::endl;
  std::cout << extrinsic_matrix << std::endl;

  //cv::Point center(res.at<float>(0, 0), res.at<float>(1, 0)); // define center of the circle

  
  key = cv::pollKey();

  if(print_once){
    cv::namedWindow("P5");
    cv::createTrackbar("Option", "P5", nullptr, max_value_choose_opt, 0);
    cv::setTrackbarPos("Option", "P5", init_value_choose_opt);
    cv::createTrackbar("Iterations", "P5", nullptr, max_value_iters, 0);
    cv::setTrackbarPos("Iterations", "P5", init_value_iters);
    cv::createTrackbar("Distance", "P5", nullptr, max_value_distance, 0);
    cv::setTrackbarPos("Distance", "P5", init_value_distance);

    //cv::Mat point_req = (cv::Mat_<float>(4,1) << 3.0, 1.4, 0, 1);

    print_once = false;
  }


  int value_choose_opt = cv::getTrackbarPos("Option", "P5");
  int value_iters = cv::getTrackbarPos("Iterations", "P5");
  //int value_distance = cv::getTrackbarPos("Distance", "P5");

  cv::setMouseCallback( "P5", on_mouse, 0 );

  switch(value_choose_opt) {

    case 0:
      std::cout << "0: Original in color\n" << std::endl;
      out_image = in_image;
      //reset image in case there are point
      points.clear(); 
      break;

    case 1:
      std::cout << "1:Detect skeleton\n" << std::endl;

      out_image = detect_skeleton(in_image, value_iters);


      for (uint i = 0; i < points.size(); i++) {
        circle(out_image, points[i], 3, cv::Scalar(0, 0, 255), -1);
      }

      std::cout << res << std::endl;
      //std::cout << res2 << std::endl;
      std::cout << out_image.size() << std::endl;

      //std::cout << res.at<float>(0, 0) << std::endl;
      //std::cout << res.at<float>(1, 0) << std::endl;
      break;

    case 2:
      std::cout << "2: Deep image\n" << std::endl;
      out_image = in_image;
      ///cv::Point center(res.at<float>(0, 0), res.at<float>(1, 0));
      cv::Point center(res.at<float>(0, 0)/res.at<float>(2, 0),res.at<float>(1, 0)/abs(res.at<float>(2, 0)));
      cv::circle(out_image,center, 3, cv::Scalar(0, 255, 255), -1); // draw the circle on the image


      cv::Point center2(res2.at<float>(0, 0)/res2.at<float>(2, 0),res2.at<float>(1, 0)/abs(res2.at<float>(2, 0)));
      cv::circle(out_image,center2, 3, cv::Scalar(0, 255, 255), -1); // draw the circle on the image

      cv::line(out_image, center, center2, cv::Scalar(0, 255, 255), 2);


      //cv::circle(out_image,center2, 3, cv::Scalar(0, 255, 255), -1); // draw the circle on the image

      //out_image = deep_image(in_image, false);

      break;
  }
  
    
  cv::imshow("P5",out_image);

  return out_image;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComputerVisionSubscriber>());
  rclcpp::shutdown();
  return 0;
}