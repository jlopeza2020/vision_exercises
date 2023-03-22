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
cv::Mat blue_balls_dt(cv::Mat in_image, bool is_opt3){

  cv::Mat img_inHSV, blue_dt, cpy_in_img, out_img;
  
  // create a clone of input image
  cpy_in_img = in_image.clone();

  // convert image in hsv 
  cv::cvtColor(in_image, img_inHSV, cv::COLOR_BGR2HSV);
  // Detect the object in green
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
}

cv::Mat green_tags_dt(cv::Mat in_image, int value_hough, bool is_opt3){

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
}


cv::Mat get_contourns(cv::Mat in_image, int value_hough, uint value_area){

  // create a clone of input image
  cv::Mat cpy_in_img = in_image.clone();
  
  cv::Mat tags_detected = green_tags_dt(in_image, value_hough, true);
  cv::Mat balls_detected = blue_balls_dt(in_image, true);

  cv::Mat and_images = tags_detected + balls_detected;
  // using bitwise does not work

  // contours 
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(and_images, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  srand(0);
  //std::cout << contours.size() << std::endl;
  //while (idx >= 0) {
  for (uint i = 0; i < contours.size(); i++) {
    //std::cout << i << std::endl;
    cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
    if (contours[i].size() > value_area) {
      cv::Moments moments = cv::moments(contours[i]);
      double cx = moments.m10 / moments.m00;
      double cy = moments.m01 / moments.m00;
      drawContours(cpy_in_img, contours, i, color, cv::LINE_4, 8, hierarchy, 1);
      circle(cpy_in_img, cv::Point(cx, cy), 4, color, -1);

      cv::String text = std::to_string(contours[i].size());
      cv::putText(cpy_in_img, text , cv::Point(cx+5, cy+5),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, color);
    }
    //drawContours(cpy_in_img, contours, i, color, cv::LINE_4, 8, hierarchy, 1);       // Last value navigates into the hierarchy
    //idx = hierarchy[idx][0];
  }

  /*for (size_t i = 0; i < contours.size(); i++) {
    cv::Scalar color(i * 10, i * 20, 0);
    cv::drawContours(cpy_in_img, contours, i, color, 5);       // Last value navigates into the hierarchy
    //idx = hierarchy[idx][0];
  }*/
  //cv::drawContours(cpy_in_img, contours, -1, cv::Scalar(0, 0, 255), 2);

  /*// Cargar las dos imágenes filtradas de las opciones 1 y 2
    Mat img1 = imread("img1.png");
    Mat img2 = imread("img2.png");

    // Fusionar las imágenes
    Mat fused_img;
    addWeighted(img1, 0.5, img2, 0.5, 0, fused_img);

    // Aplicar un algoritmo de detección de contornos
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(fused_img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // Filtrar los contornos por cantidad de puntos
    int min_contour_points = 100; // valor del slider
    vector<vector<Point>> filtered_contours;
    for (const auto& contour : contours) {
        if (contour.size() > min_contour_points) {
            filtered_contours.push_back(contour);
        }
    }

    // Calcular centroides y dibujar contornos
    Mat contour_img = Mat::zeros(fused_img.size(), CV_8UC3);
    int color_index = 0;
    for (const auto& contour : filtered_contours) {
        Moments moments = cv::moments(contour);
        double cx = moments.m10 / moments.m00;
        double cy = moments.m01 / moments.m00;

        Scalar color = Scalar(color_index % 256, (color_index / 256) % 256, (color_index / 256 / 256) % 256);
        drawContours(contour_img, vector<vector<Point>>{contour}, 0, color, 2);
        circle(contour_img, Point(cx, cy), 4, color, -1);

        cout << "Contour #" << color_index << " has " << contour.size() << " points" << endl;

        color_index++;
    }

    imshow("Contour Image", contour_img);
    waitKey(0);

    return 0;
 }*/

  // extraer contornos y centros
  // gaussian blur 

  return cpy_in_img;
}

cv::Mat get_hsv(cv::Mat in_image, int min_h, int min_s ,int min_v, int max_h, int max_s, int max_v){

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
}

cv::Mat image_processing(const cv::Mat in_image) 
{
  
  // Create output image
  cv::Mat out_image;

  int max_value_choose_opt = 3;
  int init_value_choose_opt = 0;
  int max_value_hough = 200;
  int init_value_hough = 100;
  int max_value_area = 1000;
  int init_value_area = 100;


  key = cv::pollKey();

  /*int max_h = 255;
  int max_s =  255;
  int max_v =  255;
  int min_h = 255;
  int min_s =  255;
  int min_v =  255;
  int zero = 0;*/

  if(print_once){
    cv::namedWindow("P4");
    cv::createTrackbar("0:Original; 1.Lines; 2.Balls; 3:Contours", "P4", nullptr, max_value_choose_opt, 0);
    cv::setTrackbarPos("0:Original; 1.Lines; 2.Balls; 3:Contours", "P4", init_value_choose_opt);
    cv::createTrackbar("Hough accumulator", "P4", nullptr, max_value_hough, 0);
    cv::setTrackbarPos("Hough accumulator", "P4", init_value_hough);
    cv::createTrackbar("Area", "P4", nullptr, max_value_area, 0);
    cv::setTrackbarPos("Area", "P4", init_value_area);

    /*cv::createTrackbar("max H", "P4", nullptr, max_h, 0);
    cv::setTrackbarPos("max H", "P4", zero);
    cv::createTrackbar("max S", "P4", nullptr, max_s, 0);
    cv::setTrackbarPos("max S", "P4", zero);
    cv::createTrackbar("max V", "P4", nullptr, max_v, 0);
    cv::setTrackbarPos("max V", "P4", zero);

    cv::createTrackbar("min H", "P4", nullptr, min_h, 0);
    cv::setTrackbarPos("min H", "P4", zero);
    cv::createTrackbar("min S", "P4", nullptr, min_s, 0);
    cv::setTrackbarPos("min S", "P4", zero);
    cv::createTrackbar("min V", "P4", nullptr, min_v, 0);
    cv::setTrackbarPos("min V", "P4", zero);*/

    print_once = false;
  }


  int value_choose_opt = cv::getTrackbarPos("0:Original; 1.Lines; 2.Balls; 3:Contours", "P4");
  int value_hough = cv::getTrackbarPos("Hough accumulator", "P4");
  uint value_area = cv::getTrackbarPos("Area", "P4");

  /*int gt_max_h = cv::getTrackbarPos("max H", "P4");
  int gt_max_s = cv::getTrackbarPos("max S", "P4");
  int gt_max_v = cv::getTrackbarPos("max V", "P4");
  int gt_min_h = cv::getTrackbarPos("min H", "P4");
  int gt_min_s = cv::getTrackbarPos("min S", "P4");
  int gt_min_v = cv::getTrackbarPos("min V", "P4");*/

  switch(value_choose_opt) {

    case 0:
      std::cout << "0: Original in color\n" << std::endl;
      out_image = in_image;
      break;

    case 1:
      std::cout << "1:Green tags detector\n" << std::endl;
      out_image = green_tags_dt(in_image, value_hough, false);
      //out_image = get_hsv(in_image, gt_min_h, gt_min_s ,gt_min_v, gt_max_h, gt_max_s, gt_max_v);

      break;

    case 2:
      std::cout << "2: Blue balls detector\n" << std::endl;
      out_image = blue_balls_dt(in_image, false);

      break;

    case 3:
      std::cout << "3: Get contours from opt1 and opt2\n" << std::endl;
      out_image = get_contourns(in_image, value_hough, value_area);

      break;
  }
    
  // Show image in a different window
  cv::imshow("P4",out_image);
  cv::imshow("P42",in_image);

  return out_image;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComputerVisionSubscriber>());
  rclcpp::shutdown();
  return 0;
}