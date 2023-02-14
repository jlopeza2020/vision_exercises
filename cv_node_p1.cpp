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

void image_inCMY(cv::Mat processing_image) 
{

  for ( int i=0; i < processing_image.rows; i++ ) {
    for ( int j=0; j < processing_image.cols; j++ ) { 

        // You can now access the pixel value with cv::Vec3b
        processing_image.at<cv::Vec3b>(i,j)[0] = 255 - (uint)processing_image.at<cv::Vec3b>(i,j)[0];
        processing_image.at<cv::Vec3b>(i,j)[1] = 255 - (uint)processing_image.at<cv::Vec3b>(i,j)[1];
        processing_image.at<cv::Vec3b>(i,j)[2] = 255 - (uint)processing_image.at<cv::Vec3b>(i,j)[2];

    }
  }
}

void image_inHSI(cv::Mat processing_image) 
{
  
  for ( int i=0; i < processing_image.rows; i++ ) {
    for ( int j=0; j < processing_image.cols; j++ ) { 

      double R, B, G, H, S, I;
      double pi = M_PI;

      // You can now access the pixel value with cv::Vec3b
      //  blue 
      B = (uint)processing_image.at<cv::Vec3b>(i,j)[0];
      //green
      G = (uint)processing_image.at<cv::Vec3b>(i,j)[1];
      //red
      R = (uint)processing_image.at<cv::Vec3b>(i,j)[2];

      // pixel normalized  values 
      B = B / 255.0;
      G = G / 255.0;
      R = R / 255.0;

      H = acos(1/2*((R-G) + (R-B))/sqrt((R - B)*(R - B) + (R - B)*(G - B)));

      H = H*180.0/pi; // to convert it into degrees

      if (B > G)
      {
        H = 360.0 - H;
      }
    
      S = 1 -(3.0/(R + G + B))*std::min(R, std::min(G, B)); 
 
      I = (R + G + B) / 3.0;

      // interval set into [0, 255]
      H =(H/360)*255.0;
      S = S*255.0;
      I = I*255.0;

       // H = blue
      processing_image.at<cv::Vec3b>(i,j)[0] = H;
      // S = green
      processing_image.at<cv::Vec3b>(i,j)[1] = S;
      // I = red
      processing_image.at<cv::Vec3b>(i,j)[2] = I;
        
    }
  }
}

void image_inHSV(cv::Mat processing_image) 
{
  for ( int i=0; i < processing_image.rows; i++ ) {
    for ( int j=0; j < processing_image.cols; j++ ) { 

      double R, B, G, H, S, V;
      double pi = M_PI;

      // You can now access the pixel value with cv::Vec3b
      //  blue 
      B = (uint)processing_image.at<cv::Vec3b>(i,j)[0];
      //green
      G = (uint)processing_image.at<cv::Vec3b>(i,j)[1];
      //red
      R = (uint)processing_image.at<cv::Vec3b>(i,j)[2];

      // pixel normalized  values 
      B = B / 255.0;
      G = G / 255.0;
      R = R / 255.0;

      H = acos(1/2*((R-G) + (R-B))/sqrt((R - B)*(R - B) + (R - B)*(G - B)));

      H = H*180.0/pi; // to convert it into degrees

      if (B > G)
      {
        H = 360.0 - H;
      }
    
      S = 1.0 -(3.0/(R + G + B))*std::min(R, std::min(G, B)); 
 
      V = std::max(R, std::max(G, B));

      // interval set into [0, 255]
      H =(H/360.0)*255.0;
      S = S*255.0;
      V = V*255.0;

       // H = blue
      processing_image.at<cv::Vec3b>(i,j)[0] = H;
      // S = green
      processing_image.at<cv::Vec3b>(i,j)[1] = S;
      // V = red
      processing_image.at<cv::Vec3b>(i,j)[2] = V;
        
    }
  }
  
}

//Opción 6: Mostrar la imagen en formato de color HSI utilizando la función cvtColor de
//OpenCV para obtener los canales H y S, y calculando el canal I manualmente.
void image_inHSVOP(cv::Mat processing_image) 
{
  cvtColor(processing_image, processing_image, cv::COLOR_BGR2HSV);
}


//Opción 6: Mostrar la imagen en formato de color HSI utilizando la función cvtColor de
//OpenCV para obtener los canales H y S, y calculando el canal I manualmente.
void image_inHSIOP(cv::Mat processing_image) 
{
  std::vector<cv::Mat> BGR_channels;
  std::vector<cv::Mat> HSI_channels;

  cv::split(processing_image, BGR_channels );

  cvtColor(processing_image, processing_image, cv::COLOR_BGR2HSV);
  cv::split(processing_image, HSI_channels);

  // Now I can access each channel separately
  for( int i=0; i<processing_image.rows; i++ ) {
    for( int j=0; j<processing_image.cols; j++ ) {
      // DO BY HAND I channel = Red
      float blue = BGR_channels[0].at<uchar>(i,j);
      float green = BGR_channels[1].at<uchar>(i,j);
      float red = BGR_channels[2].at<uchar>(i,j);

      // pixel normalized  values 
      blue = blue / 255.0;
      green = green / 255.0;
      red = red / 255.0;

      //double i_channel = (red + green + blue) / 3;

      //i_channel = i_channel*255;

      // I = red
      HSI_channels[2].at<uchar>(i,j) = ((red + green + blue) / 3.0)*255.0;

    }
  }

  // Create new image combining channels
  std::vector<cv::Mat> channels;
  channels.push_back(HSI_channels[0]); // H
  channels.push_back(HSI_channels[1]); // S
  channels.push_back(HSI_channels[2]);  // I

  //cv::Mat new_image;
  merge(channels, processing_image);
  //imshow("New image", new_image);

  //processing_image = new_image;

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
      std::cout << "1: RGB\n" << std::endl;
      break;

    case 50:
      last_key = 50;
      std::cout << "2: CMY\n" << std::endl;
      image_inCMY(out_image);
      break;

    case 51:
      last_key = 51;
      std::cout << "3: HSI\n" << std::endl;
      image_inHSI(out_image);
      break;

    case 52:
      last_key = 52;
      std::cout << "4: HSV\n" << std::endl;
      image_inHSV(out_image);
      break;

    case 53:
      last_key = 53;
      std::cout << "5: HSV OpenCV\n" << std::endl;
      image_inHSVOP(out_image);
      break;
      
    case 54:
      last_key = 54;
      std::cout << "6: HSI OpenCV\n" << std::endl;
      image_inHSIOP(out_image);
      break;
  }
  
  // Write text in an image
  cv::String text = "1:RGB, 2:CMY, 3:HSI, 4:HSV, 5: HSV OpenCV, 6: HSI OpenCV";
  cv::putText(out_image, text , cv::Point(10, 20),
  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

  // Show image in a different window
  cv::imshow("out_image",out_image);
  //cv::waitKey(3);

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
