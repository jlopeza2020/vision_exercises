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

cv::Mat image_gray(cv::Mat in_img)
{
  cv::Mat out_img;

  cv::cvtColor(in_img , out_img, cv::COLOR_BGR2GRAY);
  cv::cvtColor(out_img , out_img, cv::COLOR_GRAY2BGR);

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


cv::Mat image_fourier(cv::Mat input_img) 
{
  cv::Mat gray_image;

  cv::cvtColor(input_img , gray_image, cv::COLOR_BGR2GRAY);
  
  // Compute the Discrete fourier transform
  cv::Mat complexImg = computeDFT(gray_image);

  // Get the spectrum
  //cv::Mat spectrum_original = spectrum(complexImg);

  // Crop and rearrange
  //cv::Mat shift_complex = fftShift(complexImg); // Rearrange quadrants - Spectrum with low values at center - Theory mode
  //doSomethingWithTheSpectrum(shift_complex);   
  //cv::Mat rearrange = fftShift(shift_complex); // Rearrange quadrants - Spectrum with low values at corners - OpenCV mode

  // Get the spectrum after the processing
  //cv::Mat spectrum_filter = spectrum(rearrange);
  cv::Mat spectrum_filter = spectrum(complexImg);

  //out = spectrum_filter;
  return spectrum_filter;
 
  
}

// image = shiffted_complex
void get_hv_frecuencies(cv::Mat image){

  //cv::Mat filter(image.rows, image.cols, CV_64FC4, cv::Scalar(0));
  cv::Mat filter = cv::Mat::zeros(image.rows, image.cols, CV_32FC2);
  //cv::Mat result = cv::Mat::ones(image.rows, image.cols, CV_32FC2);
  int value = 25;
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

  //cv::Mat result;
  //filter(cv::Range(cy - 10, cy + 10), cv::Range(cx - 10, cx + 10)) = 1.0f;
  cv::mulSpectrums(image, filter, image, 0); // multiply 2 spectrums
  //std::cout << "image" << image.type() << std::endl;
  //std::cout << "filter" << filter.type() << std::endl;

  //cv::imshow("mask", filter);

  //cv::imshow("multiplied", image);
}

cv::Mat image_keep_filter(cv::Mat input_image) 
{
  cv::Mat gray_image;

  cv::cvtColor(input_image , gray_image, cv::COLOR_BGR2GRAY);
  
  // Compute the Discrete fourier transform
  cv::Mat complexImg = computeDFT(gray_image);

  // Crop and rearrange
  cv::Mat shift_complex = fftShift(complexImg); // Rearrange quadrants - Spectrum with low values at center - Theory mode
  //doSomethingWithTheSpectrum(shift_complex); 
  get_hv_frecuencies(shift_complex);  
  cv::Mat rearrange = fftShift(shift_complex); // Rearrange quadrants - Spectrum with low values at corners - OpenCV mode

  // Get the spectrum after the processing
  cv::Mat spectrum_filter = spectrum(rearrange);

  // Results
  //imshow("Input Image"        , I   );    // Show the result
  //imshow("Spectrum original"  , spectrum_original);
  //imshow("Spectrum filter"    , spectrum_filter);

  // Calculating the idft
  cv::Mat inverseTransform;
  cv::idft(rearrange, inverseTransform, cv::DFT_INVERSE|cv::DFT_REAL_OUTPUT);
  cv::normalize(inverseTransform, inverseTransform, 0, 1, cv::NORM_MINMAX);
  //imshow("Reconstructed", inverseTransform);


  return inverseTransform;
  //return spectrum_filter;

}

/*cv::Mat image_remove_filter(cv::Mat img) 
{
  //v::Mat img = cv::imread("imagen.jpg", cv::IMREAD_GRAYSCALE); // Leer imagen en escala de grises
  cv::Mat gray_img;

  cv::cvtColor(img , gray_img, cv::COLOR_BGR2GRAY);
  cv::Mat img_float, img_dft;
  gray_img.convertTo(img_float, CV_32FC2); // Convertir la imagen a tipo float
  cv::dft(img_float, img_dft, cv::DFT_COMPLEX_OUTPUT); // Calcular la transformada de Fourier de la imagen

  // Crear matriz filtro
  cv::Mat filter_horizontal = cv::Mat::ones(1, gray_img.rows, CV_32F);
  cv::Mat filter_vertical = cv::Mat::ones(gray_img.cols, 1, CV_32F);
  cv::Mat filter = filter_vertical * filter_horizontal;
  cv::Mat filter_mat;
  cv::merge(std::vector<cv::Mat>{filter, filter}, filter_mat);

  // Aplicar filtro al espectro de Fourier
  cv::Mat img_filtered;
  cv::mulSpectrums(img_dft, filter_mat, img_filtered, 0);

  // Calcular la transformada inversa de Fourier de la imagen filtrada
  cv::Mat img_ifft;
  img_ifft.convertTo(img_float, CV_32FC2); // Convertir la imagen a tipo float
  cv::dft(img_filtered, img_ifft, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);

  // Normalizar la imagen inversa y mostrarla
  cv::normalize(img_ifft, img_ifft, 0, 255, cv::NORM_MINMAX, CV_8U);
  //cv::imshow("Imagen inversa", img_ifft);
  //cv::waitKey(0);
  return img_ifft;
  
}*/

/*void image_logic_and(cv::Mat processing_image) 
{
}*/

cv::Mat image_processing(const cv::Mat in_image) 
{
  
  // Create output image
  cv::Mat out_image;

  out_image = in_image;

  key = cv::pollKey();

  if (key == -1){

    key = last_key;

  }

  switch(key) {
    case 49:
      last_key = 49;
      std::cout << "1: GRAY\n" << std::endl;
      // image in gray
      out_image = image_gray(in_image);
      break;

    case 50:
      last_key = 50;
      std::cout << "2: Fourier\n" << std::endl;
      
      out_image = image_fourier(in_image);
      
      // to make the headings in red
      cv::cvtColor(out_image , out_image, cv::COLOR_GRAY2BGR);
      break;

    case 51:
      last_key = 51;
      std::cout << "3: Keep Filter\n" << std::endl;
      out_image = image_keep_filter(in_image);
      break;

    case 52:
      last_key = 52;
      std::cout << "4: Remove Filter\n" << std::endl;
      //out_image = image_remove_filter(out_image);
      break;

    case 53:
      last_key = 53;
      std::cout << "5: AND\n" << std::endl;
      //image_logic_and(out_image);
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