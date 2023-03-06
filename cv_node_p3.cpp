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


/*cv::Mat image_fourier(cv::Mat input_img) 
{
  cv::Mat gray_image;

  cv::cvtColor(input_img , gray_image, cv::COLOR_BGR2GRAY);
  
  // Compute the Discrete fourier transform
  cv::Mat complexImg = computeDFT(gray_image);
  // Get the spectrum after the processing
  cv::Mat spectrum_filter = spectrum(complexImg);

  return spectrum_filter;
}*/

//image has 2 channels
/*void get_hv_frecuencies(cv::Mat image){

  cv::Mat filter = cv::Mat::zeros(image.rows, image.cols, CV_32FC2);

  int value = filter_val/2;
  int min_col = (filter.cols / 2) - value;
  int max_col = (filter.cols / 2) + value;

  int min_row = (filter.rows / 2) - value;
  int max_row = (filter.rows / 2) + value;
  
  for (int i = 0; i < filter.rows; i++){
    for (int j = 0; j < filter.cols; j++){
      if ((j > min_col && j < max_col) || (i > min_row && i < max_row)){
        filter.at<cv::Vec2f>(i,j)[0] = 1;
        filter.at<cv::Vec2f>(i,j)[1] = 1;
      }
    }
  }

  cv::mulSpectrums(image, filter, image, 0); // multiply 2 spectrums

}*/

/*cv::Mat image_keep_filter(cv::Mat input_image, bool extra) 
{
  cv::Mat gray_image;

  cv::cvtColor(input_image , gray_image, cv::COLOR_BGR2GRAY);
  
  // Compute the Discrete fourier transform
  cv::Mat complexImg = computeDFT(gray_image);

  // Crop and rearrange
  cv::Mat shift_complex = fftShift(complexImg); // Rearrange quadrants - Spectrum with low values at center - Theory mode
  // Processing vertical and horizontal frecuencies
  get_hv_frecuencies(shift_complex);  
  cv::Mat rearrange = fftShift(shift_complex); // Rearrange quadrants - Spectrum with low values at corners - OpenCV mode

  // Get the spectrum after the processing
  cv::Mat spectrum_filter = spectrum(rearrange);

  // Calculating the idft
  cv::Mat inverseTransform;
  cv::idft(rearrange, inverseTransform, cv::DFT_INVERSE|cv::DFT_REAL_OUTPUT);
  cv::normalize(inverseTransform, inverseTransform, 0, 1, cv::NORM_MINMAX);

  if (extra){
    return spectrum_filter;
  }else{
    return inverseTransform;
  }

}*/

//image has 2 channels
/*void elim_hv_frecuencies(cv::Mat image){

  cv::Mat filter = cv::Mat::zeros(image.rows, image.cols, CV_32FC2);

  int value = filter_val/2;
  int min_col = (filter.cols / 2) - value;
  int max_col = (filter.cols / 2) + value;

  int min_row = (filter.rows / 2) - value;
  int max_row = (filter.rows / 2) + value;
  
  for (int i = 0; i < filter.rows; i++){
    for (int j = 0; j < filter.cols; j++){
      if  (! ((j > min_col && j <  max_col) || (i > min_row && i < max_row))){
        filter.at<cv::Vec2f>(i,j)[0] = 1;
        filter.at<cv::Vec2f>(i,j)[1] = 1;
      }
    }
  }

  cv::mulSpectrums(image, filter, image, 0); // multiply 2 spectrums

}*/

/*cv::Mat image_remove_filter(cv::Mat input_image, bool extra)
{
  cv::Mat gray_image;

  cv::cvtColor(input_image , gray_image, cv::COLOR_BGR2GRAY);
  
  // Compute the Discrete fourier transform
  cv::Mat complexImg = computeDFT(gray_image);

  // Crop and rearrange
  cv::Mat shift_complex = fftShift(complexImg); // Rearrange quadrants - Spectrum with low values at center - Theory mode
  // Processing vertical and horizontal frecuencies
  elim_hv_frecuencies(shift_complex);  
  cv::Mat rearrange = fftShift(shift_complex); // Rearrange quadrants - Spectrum with low values at corners - OpenCV mode

  // Get the spectrum after the processing
  cv::Mat spectrum_filter = spectrum(rearrange);

  // Calculating the idft
  cv::Mat inverseTransform;
  cv::idft(rearrange, inverseTransform, cv::DFT_INVERSE|cv::DFT_REAL_OUTPUT);
  cv::normalize(inverseTransform, inverseTransform, 0, 1, cv::NORM_MINMAX);

  if (extra){
    return spectrum_filter;
  }else{
    return inverseTransform;
  }
  
}*/

//threashold set at 0.6
/*cv::Mat threshold_option3(cv::Mat src){

  cv::Mat dst(src.rows, src.cols, src.type());
  float threshold_p = 0.6;

  for (int i = 0; i < src.rows; i++){
    for (int j = 0; j < src.cols; j++){

      //just there is one channel
      if (src.at<float>(i,j) > threshold_p){
        dst.at<float>(i,j) = 1.0;
      }else{
        dst.at<float>(i,j) = 0.0;
      }

    }
  }

  return dst;
}*/

//threashold set at 0.4
/*cv::Mat threshold_option4(cv::Mat src){

  cv::Mat dst(src.rows, src.cols, src.type());
  float threshold_p = 0.4;
  
  for (int i = 0; i < src.rows; i++){
    for (int j = 0; j < src.cols; j++){

      //just there is one channel
      if (src.at<float>(i,j) > threshold_p){
        dst.at<float>(i,j) = 1.0;
      }else{
        dst.at<float>(i,j) = 0.0;
      }
    }
  }

  return dst;
}*/

/*cv::Mat image_logic_and(cv::Mat in_image) 
{
  cv::Mat out_image, op3_img, op4_img, thrs_op3, thrs_op4;

  op3_img = image_keep_filter(in_image, false);
  thrs_op3 = threshold_option3(op3_img);

  op4_img = image_remove_filter(in_image, false);
  thrs_op4 = threshold_option4(op4_img);

  bitwise_and(thrs_op3, thrs_op4, out_image);

  return out_image;
  
}*/

void low_pass_filter(cv::Mat image){

  cv::Mat filter = cv::Mat::zeros(image.rows, image.cols, CV_32FC2);

  cv::Point center(filter.cols/2, filter.rows/2);

  cv::circle(filter, center, 50, 1, -1);

  /*int value = filter_val/2;
  int min_col = (filter.cols / 2) - value;
  int max_col = (filter.cols / 2) + value;

  int min_row = (filter.rows / 2) - value;
  int max_row = (filter.rows / 2) + value;
  
  for (int i = 0; i < filter.rows; i++){
    for (int j = 0; j < filter.cols; j++){
      if  (! ((j > min_col && j <  max_col) || (i > min_row && i < max_row))){
        filter.at<cv::Vec2f>(i,j)[0] = 1;
        filter.at<cv::Vec2f>(i,j)[1] = 1;
      }
    }
  }*/

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

  cv::imshow("spectrum",spectrum_filter);

  return inverseTransform;



}

cv:: Mat image_enhaced(cv::Mat in_image){ 

  // 1. Apply low pass filter over original image in gray scale 
  cv::Mat image_low_pass = aply_filter(in_image);

  // 2. Shrink histogram using keys
  cv::Mat img_contrast;
  cv::normalize(image_low_pass, img_contrast, min_shrink_val, max_shrink_val, cv::NORM_MINMAX);

 
  // Establish the number of bins
  int histSize = 256;
  // Set the ranges (for normalized image) )
  float range[] = {0, 1};       //the upper boundary is exclusive
  const float * histRange = {range};
  bool uniform = true, accumulate = false;

  // Compute the histograms for each channel
  cv::Mat hist;
  calcHist(&img_contrast, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

  // Draw the histograms for B, G and R
  int hist_w = img_contrast.cols, hist_h = img_contrast.rows;
  int bin_w = cvRound( (double) hist_w / histSize);

  cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0) );

  // normalize the histograms between 0 and histImage.rows
  cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

  // Draw the intensity line for histograms
  for (int i = 1; i < histSize; i++) {
    cv::line(
      histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1)) ),
      cv::Point(bin_w * (i), hist_h - cvRound(hist.at<float>(i)) ),
      cv::Scalar(255, 0, 0), 2, 8, 0);
  }

  // Show images
  cv::imshow("calcHist Source", histImage);
  return img_contrast;
}

cv::Mat image_processing(const cv::Mat in_image) 
{
  
  // Create output image
  cv::Mat out_image;
  
  
  // remove_filter, keep_filter, thrs_op4, thrs_op3, op4_img, op3_img
  //bool show_ft = false;

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
        if (min_shrink_val < max_shrink_val){
          min_shrink_val -= 1;
        }

      }

      break;

    //x key: increments min value
    case 120:
    // is used only when option 3 is displaying
      if (51 == last_key){
        // show option 3
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
        if (min_shrink_val < max_shrink_val){
          max_shrink_val += 1;
        }
      }
      break;
  }
  
  // Write text in an image
  cv::String text1 = "1:Original, 2:Gray, 3:Enhaced | shrink [z-,x+]: min | [c-,v+]: max";
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