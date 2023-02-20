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

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int key;
int last_key;

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
    
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "cv_image", qos);
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

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
   
};

/**
  TO-DO COMPLETE THIS PART 
**/

void image_gray(cv::Mat processing_image)
{
    cv::cvt
}
void image_fourier(cv::Mat processing_image) 
{

  
}

void image_keep_filter(cv::Mat processing_image) 
{
  
  
}

void image_remove_filter(cv::Mat processing_image) 
{
  
  
}

void image_logic_and(cv::Mat processing_image) 
{
}

cv::Mat image_processing(const cv::Mat in_image) 
{
  
  // Create output image
  cv::Mat out_image;

  out_image = in_image;

  // swith case con las 6 opciones
  key = cv::pollKey();

  if (key == -1){

    key = last_key;

  }

  switch(key) {
    case 49:
      last_key = 49;
      std::cout << "1: GRAY\n" << std::endl;
      image_gray(out_image);
      break;

    case 50:
      last_key = 50;
      std::cout << "2: Fourier\n" << std::endl;
      image_fourier(out_image);
      break;

    case 51:
      last_key = 51;
      std::cout << "3: Keep Filter\n" << std::endl;
      image_keep_filter(out_image);
      break;

    case 52:
      last_key = 52;
      std::cout << "4: Remove Filter\n" << std::endl;
      image_remove_filter(out_image);
      break;

    case 53:
      last_key = 53;
      std::cout << "5: AND\n" << std::endl;
      image_logic_and(out_image);
      break;
  }
  
  // Write text in an image
  cv::String text1 = "1:GRAY, 2:Fourier, 3:Keep Filter, 4:Remove Filter, 5: AND";
  cv::String text2 = "[z,x]: -+ filter val: 50";
  cv::putText(out_image, text1 , cv::Point(10, 20),
  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
  cv::putText(out_image, text2 , cv::Point(10, 40),
  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

  // Show image in a different window
  cv::imshow("out_image",out_image);

  return out_image;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComputerVisionSubscriber>());
  rclcpp::shutdown();
  return 0;
}