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
int min_shrink_val = 0;
int max_shrink_val = 30;
//int d_times;

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

cv::Mat image_gray(cv::Mat in_img)
{
  cv::Mat out_img;

  cv::cvtColor(in_img , out_img, cv::COLOR_BGR2GRAY);

  return out_img;
}

// Compute the Discrete fourier transform
cv::Mat computeDFT(const cv::Mat &image) {
  // Expand the image to an optimal size. 
  cv::Mat padded;                      
  int m = cv::getOptimalDFTSize( image.rows );
  int n = cv::getOptimalDFTSize( image.cols ); // on the border add zero values
  copyMakeBorder(image, padded, 0, m - image.rows, 0, n - image.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    
  // Make place for both the complex and the real values
  cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
  cv::Mat complexI;
  merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

  // Make the Discrete Fourier Transform
  dft(complexI, complexI, cv::DFT_COMPLEX_OUTPUT);      // this way the result may fit in the source matrix
  return complexI;
}

// 6. Crop and rearrange
cv::Mat fftShift(const cv::Mat &magI) {
  cv::Mat magI_copy = magI.clone();
  // crop the spectrum, if it has an odd number of rows or columns
  magI_copy = magI_copy(cv::Rect(0, 0, magI_copy.cols & -2, magI_copy.rows & -2));
    
  // rearrange the quadrants of Fourier image  so that the origin is at the image center
  int cx = magI_copy.cols/2;
  int cy = magI_copy.rows/2;

  cv::Mat q0(magI_copy, cv::Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
  cv::Mat q1(magI_copy, cv::Rect(cx, 0, cx, cy));  // Top-Right
  cv::Mat q2(magI_copy, cv::Rect(0, cy, cx, cy));  // Bottom-Left
  cv::Mat q3(magI_copy, cv::Rect(cx, cy, cx, cy)); // Bottom-Right

  cv::Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
  q0.copyTo(tmp);
  q3.copyTo(q0);
  tmp.copyTo(q3);

  q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
  q2.copyTo(q1);
  tmp.copyTo(q2);

  return magI_copy;
}

// Calculate dft spectrum
cv::Mat spectrum(const cv::Mat &complexI) {

  cv::Mat complexImg = complexI.clone();
  // Shift quadrants
  cv::Mat shift_complex = fftShift(complexImg);

  // Transform the real and complex values to magnitude
  // compute the magnitude and switch to logarithmic scale
  // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
  cv::Mat planes_spectrum[2];
  split(shift_complex, planes_spectrum);       // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
  magnitude(planes_spectrum[0], planes_spectrum[1], planes_spectrum[0]);// planes[0] = magnitude
  cv::Mat spectrum = planes_spectrum[0];

  // Switch to a logarithmic scale
  spectrum += cv::Scalar::all(1);
  log(spectrum, spectrum);

  // Normalize
  normalize(spectrum, spectrum, 0, 1, cv::NORM_MINMAX); // Transform the matrix with float values into a
                                                      // viewable image form (float between values 0 and 1).
  return spectrum;
}

void low_pass_filter(cv::Mat image){

  cv::Mat filter = cv::Mat::zeros(image.rows, image.cols, CV_32FC2);

  cv::Point center(filter.cols/2, filter.rows/2);

  cv::circle(filter, center, 50, 1, -1);

  cv::mulSpectrums(image, filter, image, 0); // multiply 2 spectrums

}

cv::Mat aply_filter(cv::Mat in_image){

  //set image in gray
  cv::Mat gray_image;

  cv::cvtColor(in_image , gray_image, cv::COLOR_BGR2GRAY);

   // Compute the Discrete fourier transform
  cv::Mat complexImg = computeDFT(gray_image);

  // Crop and rearrange
  cv::Mat shift_complex = fftShift(complexImg); // Rearrange quadrants - Spectrum with low values at center - Theory mode
  // Processing vertical and horizontal frecuencies
  low_pass_filter(shift_complex);  
  cv::Mat rearrange = fftShift(shift_complex); // Rearrange quadrants - Spectrum with low values at corners - OpenCV mode

  // Get the spectrum after the processing
  cv::Mat spectrum_filter = spectrum(rearrange);

  // Calculating the idft
  cv::Mat inverseTransform;
  cv::idft(rearrange, inverseTransform, cv::DFT_INVERSE|cv::DFT_REAL_OUTPUT);
  cv::normalize(inverseTransform, inverseTransform, 0, 1, cv::NORM_MINMAX);

  return inverseTransform;
}

cv::Mat shrink_histogram(cv::Mat image){

  cv::Mat dst(image.rows, image.cols, CV_32FC1, cv::Scalar(0.0));
  float r_min = 0.0;
  float r_max = 255.0;
  
  for (int i = 0; i < image.rows; i++){
    for (int j = 0; j < image.cols; j++){
      dst.at<float>(i,j) = ((max_shrink_val - min_shrink_val)/(r_max - r_min))*(image.at<float>(i,j) - r_min) + min_shrink_val;
      //std::cout << dst.at<float>(i,j) << std::endl;
    }
  }

  return dst;
}

cv::Mat expand_image(cv::Mat image){

  cv::Mat dst(image.rows, image.cols, CV_32FC1, cv::Scalar(0.0));
  float min = 0.0;
  float max = 255.0;
  
  for (int i = 0; i < image.rows; i++){
    for (int j = 0; j < image.cols; j++){

      //std::cout << image.at<float>(i,j) << std::endl;
      dst.at<float>(i,j) = ((image.at<float>(i,j)- 0.0)/(1.0 - 0.0))*(max - min) + min;
    }
  }

  return dst;

}
cv::Mat image_enhaced(cv::Mat in_image){ 

  // 1. Apply low pass filter over original image in gray scale 
  cv::Mat image_low_pass = aply_filter(in_image);

  // 2. Shrink histogram using keys
  cv::Mat image_shrinked = shrink_histogram(image_low_pass);

  // 3. Substract pixel to pixel image got in 
  cv::Mat image_substracted(in_image.rows, in_image.cols, CV_32FC1, cv::Scalar(0.0));
  cv::Mat gray_image(in_image.rows, in_image.cols, CV_32FC1, cv::Scalar(0.0));
  cv::cvtColor(in_image , gray_image, cv::COLOR_BGR2GRAY);
  cv::subtract(gray_image, image_shrinked, image_substracted, cv::noArray(), CV_32FC1);

  // I tried using this function made by myself but it didn't work 
  //cv::Mat image_expanded = expand_image(image_substracted);
  cv::Mat image_expanded;
  cv::normalize(image_substracted, image_expanded, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  //cv::normalize(image_substracted, image_expanded, 0, 255, cv::NORM_MINMAX, CV_32FC1);


  // 5. Equalized image from 4 
  //cv::Mat image_eq, expanded_in_8u;
  cv::Mat image_eq;

  //image_expanded.convertTo(expanded_in_8u, CV_8UC1);

  //cv::equalizeHist(expanded_in_8u, image_eq);
  cv::equalizeHist(image_expanded, image_eq);




  // CREATE HISTOGRAM
  // Establish the number of bins
  int histSize = 256;

  float range[] = {0, 255};       //the upper boundary is exclusive
  const float * histRange = {range};
  bool uniform = true, accumulate = false;

  cv::Mat hist_shrinked, hist_substracted, hist_expanded, hist_eq;
  calcHist(&image_shrinked, 1, 0, cv::Mat(), hist_shrinked, 1, &histSize, &histRange, uniform, accumulate);
  calcHist(&image_substracted, 1, 0, cv::Mat(), hist_substracted, 1, &histSize, &histRange, uniform, accumulate);
  calcHist(&image_expanded, 1, 0, cv::Mat(), hist_expanded, 1, &histSize, &histRange, uniform, accumulate);
  calcHist(&image_eq, 1, 0, cv::Mat(), hist_eq, 1, &histSize, &histRange, uniform, accumulate);

  // Draw the histograms for B, G and R
  int hist_w = in_image.cols, hist_h = in_image.rows;
  int bin_w = cvRound((double) hist_w / histSize);

  cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0) );

  // normalize the histograms between 0 and histImage.rows
  cv::normalize(hist_shrinked, hist_shrinked, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
  cv::normalize(hist_substracted, hist_substracted, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
  cv::normalize(hist_expanded, hist_expanded, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
  cv::normalize(hist_eq, hist_eq, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );


  // Draw the intensity line for histograms
  for (int i = 1; i < histSize; i++) {
    cv::line(
      histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(hist_shrinked.at<float>(i - 1)) ),
      cv::Point(bin_w * (i), hist_h - cvRound(hist_shrinked.at<float>(i)) ),
      cv::Scalar(0, 0, 255), 2, 8, 0);
    
    cv::line(
      histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(hist_substracted.at<float>(i - 1)) ),
      cv::Point(bin_w * (i), hist_h - cvRound(hist_substracted.at<float>(i)) ),
      cv::Scalar(255, 255, 0), 2, 8, 0);

    cv::line(
      histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(hist_expanded.at<float>(i - 1)) ),
      cv::Point(bin_w * (i), hist_h - cvRound(hist_expanded.at<float>(i)) ),
      cv::Scalar(0, 255, 255), 2, 8, 0);

    cv::line(
      histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(hist_eq.at<float>(i - 1)) ),
      cv::Point(bin_w * (i), hist_h - cvRound(hist_eq.at<float>(i)) ),
      cv::Scalar(0, 128, 0), 2, 8, 0);
  }

  // Show images
  // add legend 
  cv::imshow("calcHist Source", histImage);

  //cv::imshow("contracted", image_shrinked);
  //cv::imshow("substracted", image_substracted);
  //cv::imshow("expanded", image_expanded);

  return image_eq;
  //return image_expanded;



}

cv::Mat image_processing(const cv::Mat in_image) 
{
  
  // Create output image
  cv::Mat out_image;
  out_image = in_image;

  key = cv::pollKey();

  if (key == -1){

    key = last_key;

  }

  // I am using ASCII code
  switch(key) {
    // Option 1
    case 49:
      last_key = 49;
      std::cout << "1: Original in color\n" << std::endl;
      break;

    // Option 2
    case 50:
      last_key = 50;
      std::cout << "2: Original in GRAY\n" << std::endl;
      out_image = image_gray(in_image);

      break;

    // Option 3
    case 51:
      last_key = 51;
      std::cout << "3: Enhaced\n" << std::endl;
      out_image = image_enhaced(in_image);

      // make the headings in red
      cv::cvtColor(out_image , out_image, cv::COLOR_GRAY2BGR);
      break;

    //z key: decrements min value 
    case 122:
      // is used only when option 3 is displaying
      if (51 == last_key){
        // show option 3
        out_image = image_enhaced(in_image);
        // make the headings in red
        cv::cvtColor(out_image , out_image, cv::COLOR_GRAY2BGR);

        if ( 0 < min_shrink_val && min_shrink_val < max_shrink_val){
          min_shrink_val -= 1;
        }
      }
      break;

    //x key: increments min value
    case 120:
    // is used only when option 3 is displaying
      if (51 == last_key){
        // show option 3
        out_image = image_enhaced(in_image);
        // make the headings in red
        cv::cvtColor(out_image , out_image, cv::COLOR_GRAY2BGR);

        if (min_shrink_val + 1 < max_shrink_val){
          min_shrink_val += 1;
        }
      }
      break;

    //c key: decrements max value 
    case 99:
      // is used only when option 3 is displaying
      if (51 == last_key){
        // show option 3
        out_image = image_enhaced(in_image);
        // make the headings in red
        cv::cvtColor(out_image , out_image, cv::COLOR_GRAY2BGR);

        if (min_shrink_val < max_shrink_val - 1){
          max_shrink_val -= 1;
        }
      }
      break;

    //v key: increments max value
    case 118:
    // is used only when option 3 is displaying
      if (51 == last_key){
        // show option 3
        out_image = image_enhaced(in_image);
        // make the headings in red
        cv::cvtColor(out_image , out_image, cv::COLOR_GRAY2BGR);

        if (min_shrink_val < max_shrink_val  && max_shrink_val < 255){
          max_shrink_val += 1;
        }
      }
      break;
  }
  
  // Write text in an image
  cv::String text1 = "1: Original, 2: Gray, 3: Enhaced | shrink [z-,x+]: min | [c-,v+]: max";
  cv::String text2 = "shrink [min: " + std::to_string(min_shrink_val) + ", max: "+ std::to_string(max_shrink_val)+"]";
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