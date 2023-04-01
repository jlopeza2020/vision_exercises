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
bool print_once = true;

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

// usar momenets y contors 
/*cv::Mat blue_balls_dt(cv::Mat in_image, bool is_opt3){

  cv::Mat img_inHSV, blue_dt, cpy_in_img, out_img;
  
  // create a clone of input image
  cpy_in_img = in_image.clone();

  // convert image in hsv 
  cv::cvtColor(in_image, img_inHSV, cv::COLOR_BGR2HSV);
  // Detect the object in blue
  cv::inRange(img_inHSV, cv::Scalar(94, 100, 23), cv::Scalar(126,255,255), blue_dt);

  // Edge detection
  Canny(blue_dt, out_img, 50, 200, 3);

  std::vector<cv::Vec3f> circles;
  HoughCircles(
    out_img, circles, cv::HOUGH_GRADIENT, 1,
    out_img.rows / 160,             // change this value to detect circles with different distances to each other
    200, 30, 1, 10000              // change the last two parameters (min_radius & max_radius) to detect larger circles
  );

  for (size_t i = 0; i < circles.size(); i++) {
    cv::Vec3i c = circles[i];
    cv::Point center = cv::Point(c[0], c[1]);
    // circle center
    cv::circle(cpy_in_img, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
    // circle outline
    int radius = c[2];
    cv::circle(cpy_in_img, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
  }

  if(is_opt3){
    return out_img;
  }else{
    return cpy_in_img;
  }
}*/

/*cv::Mat green_tags_dt(cv::Mat in_image, int value_hough, bool is_opt3){

  cv::Mat img_inHSV,out_img, green_dt, cpy_in_img;

  // create a clone of input image
  cpy_in_img = in_image.clone();

  // convert image in hsv 
  cv::cvtColor(in_image, img_inHSV, cv::COLOR_BGR2HSV);
  // Detect the object in green
  cv::inRange(img_inHSV, cv::Scalar(64, 58, 25), cv::Scalar(80,255,255), green_dt);

  // Edge detection
  Canny(green_dt, out_img, 50, 200, 3);

  std::vector<cv::Vec2f> lines;   // will hold the results of the detection (rho, theta)
  HoughLines(out_img, lines, 1, CV_PI / 180, value_hough, 0, 0);   // runs the actual detection

  // Draw the lines
  for (size_t i = 0; i < lines.size(); i++) {
    float rho = lines[i][0], theta = lines[i][1];
    cv::Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * ( a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * ( a));
    line(cpy_in_img, pt1, pt2, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
  }

  if(is_opt3){
    return out_img;
  }else{
    return cpy_in_img;
  }
}*/


/*cv::Mat get_contourns(cv::Mat in_image, int value_hough, uint value_area){

  // create a clone of input image
  cv::Mat cpy_in_img = in_image.clone();
  
  cv::Mat tags_detected = green_tags_dt(in_image, value_hough, true);
  cv::Mat balls_detected = blue_balls_dt(in_image, true);

  // make an and of both binary images (I tried using bitwise_and and it did not work)
  cv::Mat and_images = tags_detected + balls_detected;

  // contours 
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(and_images, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  srand(0);
  for (uint i = 0; i < contours.size(); i++) {
    // set random color
    cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
    if (contours[i].size() > value_area) {

      //calculate centroide of the contour using moments
      cv::Moments moments = cv::moments(contours[i]);
      double cx = moments.m10 / moments.m00;
      double cy = moments.m01 / moments.m00;
      drawContours(cpy_in_img, contours, i, color, cv::LINE_4, 8, hierarchy, 1);
      circle(cpy_in_img, cv::Point(cx, cy), 4, color, -1);

      //print number of pixels of contour
      cv::String text = std::to_string(contourArea(contours[i]));
      cv::putText(cpy_in_img, text , cv::Point(cx+5, cy+5),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, color);
    }
  }

  return cpy_in_img;
}*/
// create mouse callback

std::vector<cv::Point> points;

void on_mouse(int event, int x, int y, int, void*)
{
  //switch (event){

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

cv::Mat detect_skeleton(cv::Mat in_image){


  cv::Mat out_image, img_inHSV;

  //out_image = in_image;

  cv::cvtColor(in_image, img_inHSV, cv::COLOR_BGR2HSV);
  // Detect the object in green
  cv::inRange(img_inHSV, cv::Scalar(0, 0, 109), cv::Scalar(255,255,117), out_image);

  for (uint i = 0; i < points.size(); i++) {
    circle(out_image, points[i], 3, cv::Scalar(0, 0, 255), -1);
  }

  return out_image;


}

/*cv::Mat get_hsv(cv::Mat in_image, int min_h, int min_s ,int min_v, int max_h, int max_s, int max_v){

  cv::Mat img_inHSV, green_dt;


  // convert image in hsv 
  cv::cvtColor(in_image, img_inHSV, cv::COLOR_BGR2HSV);
  // Detect the object in green
  cv::inRange(img_inHSV, cv::Scalar(min_h, min_s, min_v), cv::Scalar(max_h,max_s,max_v), green_dt);

  // Edge detection
  //Canny(green_dt, out_img, 50, 200, 3);

  //std::vector<cv::Vec2f> lines;   // will hold the results of the detection (rho, theta)
  //HoughLines(out_img, lines, 1, CV_PI / 180, value_hough, 0, 0);   // runs the actual detection

  return green_dt;
}*/



cv::Mat image_processing(const cv::Mat in_image) 
{
  cv::Mat out_image;

  int max_value_choose_opt = 2;
  int init_value_choose_opt = 0;
  int max_value_iters = 100;
  int init_value_iters = 20;
  int max_value_distance = 8;
  int init_value_distance = 0;

  /*int max_h = 255;
  int max_s =  255;
  int max_v =  255;
  int min_h = 255;
  int min_s =  255;
  int min_v =  255;
  int zero = 0;*/


  key = cv::pollKey();

  if(print_once){
    cv::namedWindow("P5");
    cv::createTrackbar("Option", "P5", nullptr, max_value_choose_opt, 0);
    cv::setTrackbarPos("Option", "P5", init_value_choose_opt);
    cv::createTrackbar("Iterations", "P5", nullptr, max_value_iters, 0);
    cv::setTrackbarPos("Iterations", "P5", init_value_iters);
    cv::createTrackbar("Distance", "P5", nullptr, max_value_distance, 0);
    cv::setTrackbarPos("Distance", "P5", init_value_distance);

    /*cv::createTrackbar("max H", "P5", nullptr, max_h, 0);
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
    cv::setTrackbarPos("min V", "P5", zero);*/
    print_once = false;
  }


  int value_choose_opt = cv::getTrackbarPos("Option", "P5");
  //int value_iters = cv::getTrackbarPos("Iterations", "P5");
  //int value_distance = cv::getTrackbarPos("Distance", "P5");

  /*int gt_max_h = cv::getTrackbarPos("max H", "P5");
  int gt_max_s = cv::getTrackbarPos("max S", "P5");
  int gt_max_v = cv::getTrackbarPos("max V", "P5");
  int gt_min_h = cv::getTrackbarPos("min H", "P5");
  int gt_min_s = cv::getTrackbarPos("min S", "P5");
  int gt_min_v = cv::getTrackbarPos("min V", "P5");*/

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

      //out_image = get_hsv(in_image, gt_min_h, gt_min_s ,gt_min_v, gt_max_h, gt_max_s, gt_max_v);
      //out_image = in_image;
      //cv::setMouseCallback( "P5", on_mouse, 0 );

      //for (uint i = 0; i < points.size(); i++) {
      //  circle(out_image, points[i], 3, cv::Scalar(0, 0, 255), -1);
      //}

      out_image = detect_skeleton(in_image);

      break;

    case 2:
      std::cout << "2: Deep image\n" << std::endl;
      out_image = in_image;
      //out_image = deep_image(in_image, false);

      break;
  }
    
  cv::imshow("P5",out_image);
  cv::imshow("P52",in_image);

  return out_image;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComputerVisionSubscriber>());
  rclcpp::shutdown();
  return 0;
}