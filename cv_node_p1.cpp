/*
# Copyright (c) 2022 José Miguel Guerrero Hernández
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
*/
/*cv::Mat image_inRGB(const cv::Mat image2process) 
{
  cv::Mat out_image;

}

cv::Mat image_inCMY(const cv::Mat image2process) 
{
  cv::Mat out_image;

}

cv::Mat image_inHSI(const cv::Mat image2process) 
{
  cv::Mat out_image;
}

cv::Mat image_inHSV(const cv::Mat image2process) 
{
  cv::Mat out_image;
}

cv::Mat image_inHSVOP(const cv::Mat image2process) 
{
  cv::Mat out_image;
}

cv::Mat image_inHSIOP(const cv::Mat image2process) 
{
  cv::Mat out_image;
}*/

cv::Mat image_processing(const cv::Mat in_image) 
{
  
  // Create output image
  cv::Mat out_image;
  
  // swith case con las 6 opciones
  int key = cv::pollKey();
  switch(key) {
    case 49:
      std::cout << "pulsado 1: RGB\n" << std::endl;
      //out_image = image_inRGB(in_image);
      break;
    case 50:
      std::cout << "pulsado 2: CMY\n" << std::endl;
      //out_image = image_inCMY(in_image);
      break;
    case 51:
      std::cout << "pulsado 3: HSI\n" << std::endl;
      //out_image = image_inHSI(in_image);
      break;
    case 52:
      std::cout << "pulsado 4: HSV\n" << std::endl;
      //out_image = image_inHSV(in_image);
      break;
    case 53:
      std::cout << "pulsado 5: HSV OpenCV\n" << std::endl;
      //out_image = image_inHSVOP(in_image);
      break;
    case 54:
      std::cout << "pulsado 6: HSI OpenCV\n" << std::endl;
      //out_image = image_inHSIOP(in_image);
      break;
  }

  
  // Processing (output image = input image)
  out_image = in_image;
  
  // Write text in an image
  cv::String text = "1:RGB, 2:CMY, 3:HSI, 4:HSV, 5: HSV OpenCV, 6: HSI OpenCV";
  cv::putText(out_image, text , cv::Point(10, 20),
  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

  // Show image in a different window
  cv::imshow("out_image",out_image);
  cv::waitKey(3);

  // You must to return a 3-channels image to show it in ROS, so do it with 1-channel images
  //cv::cvtColor(out_image, out_image, cv::COLOR_GRAY2BGR);
  return out_image;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComputerVisionSubscriber>());
  rclcpp::shutdown();
  return 0;
}