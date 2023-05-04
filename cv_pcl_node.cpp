/*
Autora: Julia López Augusto

Partes implementadas: 

- 
- 
-
- Funcionalidad extra: 

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
#include <opencv2/dnn.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <fstream>
#include <sstream>
#include <iostream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud);
cv::Mat image_processing(const cv::Mat in_image);

int value_choose_opt;
int value_distance;
int key;
bool print_once = true;

// Initialize the parameters
float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
int inpWidth = 416;  // Width of network's input image
int inpHeight = 416; // Height of network's input image
std::vector<std::string> classes;

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

class PCLSubscriber : public rclcpp::Node
{
  public:
    PCLSubscriber()
    : Node("pcl_subscriber")
    {
      auto qos = rclcpp::QoS( rclcpp::QoSInitialization( RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5 ));
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

      subscription_3d_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/head_front_camera/depth_registered/points", qos, std::bind(&PCLSubscriber::topic_callback_3d, this, std::placeholders::_1));
    
      publisher_3d_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "pcl_points", qos);
    }

  private:
    void topic_callback_3d(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {    
      // Convert to PCL data type
      pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
      pcl::fromROSMsg(*msg, point_cloud);     

      pcl::PointCloud<pcl::PointXYZRGB> pcl_pointcloud = pcl_processing(point_cloud);
      
      // Convert to ROS data type
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(pcl_pointcloud, output);
      output.header = msg->header;

      // Publish the data
      publisher_3d_ -> publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_3d_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_3d_;
};


/**
  TO-DO
*/

// Draw the predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat & frame)
{
  //Draw a rectangle displaying the bounding box
  rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);

  //Get the label for the class name and its confidence
  std::string label = cv::format("%.2f", conf);
  if(classId == 0){
    if (!classes.empty()) {
      CV_Assert(classId < (int)classes.size());
      label = classes[classId] + ":" + label;
    }

    //Display the label at the top of the bounding box
    int baseLine;
    cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = std::max(top, labelSize.height);
    rectangle(
      frame, cv::Point(left, top - round(1.5 * labelSize.height)),
    cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);

  }

  //Display the label at the top of the bounding box
  //int baseLine;
  //cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
  //top = std::max(top, labelSize.height);
  //rectangle(
  //  frame, cv::Point(left, top - round(1.5 * labelSize.height)),
  //  cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
  //cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
}

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(cv::Mat & frame, const std::vector<cv::Mat> & outs)
{
  std::vector<int> classIds;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;

  for (size_t i = 0; i < outs.size(); ++i) {
    // Scan through all the bounding boxes output from the network and keep only the
    // ones with high confidence scores. Assign the box's class label as the class
    // with the highest score for the box.
    float * data = (float *)outs[i].data;
    for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
      cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
      cv::Point classIdPoint;
      double confidence;
      // Get the value and location of the maximum score
      minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
      if (confidence > confThreshold) {
        int centerX = (int)(data[0] * frame.cols);
        int centerY = (int)(data[1] * frame.rows);
        int width = (int)(data[2] * frame.cols);
        int height = (int)(data[3] * frame.rows);
        int left = centerX - width / 2;
        int top = centerY - height / 2;

        classIds.push_back(classIdPoint.x);
        confidences.push_back((float)confidence);
        boxes.push_back(cv::Rect(left, top, width, height));
      }
    }
  }

  // Perform non maximum suppression to eliminate redundant overlapping boxes with
  // lower confidences
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
  for (size_t i = 0; i < indices.size(); ++i) {
    int idx = indices[i];
    cv::Rect box = boxes[idx];
    drawPred(
      classIds[idx], confidences[idx], box.x, box.y,
      box.x + box.width, box.y + box.height, frame);
  }
}


// Get the names of the output layers
std::vector<std::string> getOutputsNames(const cv::dnn::Net & net)
{
  static std::vector<std::string> names;
  if (names.empty()) {
    //Get the indices of the output layers, i.e. the layers with unconnected outputs
    std::vector<int> outLayers = net.getUnconnectedOutLayers();

    //get the names of all the layers in the network
    std::vector<std::string> layersNames = net.getLayerNames();

    // Get the names of the output layers in names
    names.resize(outLayers.size());
    for (size_t i = 0; i < outLayers.size(); ++i) {
      names[i] = layersNames[outLayers[i] - 1];
    }
  }
  return names;
}

void detect_person(cv::Mat image){

  //using namespace cv;
  //using namespace dnn;
  //using namespace std;

  // Initialize the parameters
  //float confThreshold = 0.5; // Confidence threshold
  //float nmsThreshold = 0.4;  // Non-maximum suppression threshold
  //int inpWidth = 416;  // Width of network's input image
  //int inpHeight = 416; // Height of network's input image
  //std::vector<std::string> classes;

  // Remove the bounding boxes with low confidence using non-maxima suppression
  //void postprocess(Mat & frame, const vector<Mat> & out);

  // Draw the predicted bounding box
  //void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat & frame);

  // Get the names of the output layers
  //vector<String> getOutputsNames(const Net & net);

  //int main(int argc, char ** argv)
  //{
  //CommandLineParser parser(argc, argv, keys);
  //parser.about("Use this script to run object detection using YOLO3 in OpenCV.");
  //if (parser.has("help")) {
  //  parser.printMessage();
  //  return 0;
  //}
  // Load names of classes
  std::string classesFile = "cfg/coco.names";
  std::ifstream ifs(classesFile.c_str());
  //std::string line;
  //while (getline(ifs, line)) {classes.push_back(line);}
  classes.push_back("person"); // global vector

  //std::string device = "cpu";
  //device = parser.get<std::string>("device");

  // Give the configuration and weight files for the model
  std::string modelConfiguration = "/home/juloau/Desktop/vision/ros2_computer_vision/src/computer_vision/src/cfg/yolov3.cfg";
  std::string modelWeights = "/home/juloau/Desktop/vision/ros2_computer_vision/src/computer_vision/src/cfg/yolov3.weights";

  // Load the network
  cv::dnn::Net net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);

  //if (device == "cpu") {
  //  cout << "Using CPU device" << endl;
  net.setPreferableBackend(cv::dnn::DNN_TARGET_CPU);
  //} else if (device == "gpu") {
  //  cout << "Using GPU device" << endl;
  //  net.setPreferableBackend(DNN_BACKEND_CUDA);
  //  net.setPreferableTarget(DNN_TARGET_CUDA);
  //}


  // Open a video file or an image file or a camera stream.
  //string str, outputFile;
  //VideoCapture cap;
  //VideoWriter video;
  cv::Mat frame, blob;

  frame = image;

  /*try {

    outputFile = "yolo_out_cpp.avi";
    if (parser.has("image")) {
      // Open the image file
      str = parser.get<String>("image");
      ifstream ifile(str);
      if (!ifile) {throw("error");}
      cap.open(str);
      str.replace(str.end() - 4, str.end(), "_yolo_out_cpp.jpg");
      outputFile = str;
    } else if (parser.has("video")) {
      // Open the video file
      str = parser.get<String>("video");
      ifstream ifile(str);
      if (!ifile) {throw("error");}
      cap.open(str);
      str.replace(str.end() - 4, str.end(), "_yolo_out_cpp.avi");
      outputFile = str;
    }
    // Open the webcaom
    else {cap.open(parser.get<int>("device"));}

  } catch (...) {
    cout << "Could not open the input image/video stream" << endl;
    return 0;
  }*/

  // Get the video writer initialized to save the output video
  /*if (!parser.has("image")) {
    video.open(
      outputFile, VideoWriter::fourcc('M', 'J', 'P', 'G'), 28,
      Size(cap.get(CAP_PROP_FRAME_WIDTH), cap.get(CAP_PROP_FRAME_HEIGHT)));
  }*/

  // Create a window
  //static const string kWinName = "Deep learning object detection in OpenCV";
  //namedWindow(kWinName, WINDOW_NORMAL);

  // Process frames.
  //while (cv::waitKey(1) < 0) {
    // get frame from the video
    //cap >> frame;

    // Stop the program if reached end of video
    //if (frame.empty()) {
    //  std::cout << "Done processing !!!" << std::endl;
      //std::cout << "Output file is stored as " << outputFile << std::endl;
    //  cv::waitKey(3000);
    //  break;
    //}
    // Create a 4D blob from a frame.
  cv::dnn::blobFromImage(
  frame, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(
  0, 0,0), true, false);

    //Sets the input to the network
  net.setInput(blob);

    // Runs the forward pass to get output of the output layers
  std::vector<cv::Mat> outs;
  net.forward(outs, getOutputsNames(net));

    // Remove the bounding boxes with low confidence
  postprocess(frame, outs);

    // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
  std::vector<double> layersTimes;
  double freq = cv::getTickFrequency() / 1000;
  double t = net.getPerfProfile(layersTimes) / freq;
  std::string label = cv::format("Inference time for a frame : %.2f ms", t);
  cv::putText(frame, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

    // Write the frame with the detection boxes
    //cv::Mat detectedFrame;
    //frame.convertTo(detectedFrame, CV_8U);
    //if (parser.has("image")) {imwrite(outputFile, detectedFrame);} else {
    //  video.write(detectedFrame);
    //}

    //imshow(kWinName, frame);

  //}

  //cap.release();
  //if (!parser.has("image")) {video.release();}

  //return 0;
//}

}

cv::Mat image_processing(const cv::Mat in_image) 
{
  // Create output image
  //cv::Mat out_image = in_image;

  // You must to return a 3-channels image to show it in ROS, so do it with 1-channel images
  //cv::cvtColor(out_image, out_image, cv::COLOR_GRAY2BGR);
  //return out_image;
  cv::Mat out_image;

  int max_value_choose_opt = 2;
  int init_value_choose_opt = 0;

  int max_value_distance = 8;
  int init_value_distance = 3;


  key = cv::pollKey();

  if(print_once){
    cv::namedWindow("PRACTICA_FINAL");
    cv::createTrackbar("Option", "PRACTICA_FINAL", nullptr, max_value_choose_opt, 0);
    cv::setTrackbarPos("Option", "PRACTICA_FINAL", init_value_choose_opt);
    cv::createTrackbar("Distance", "PRACTICA_FINAL", nullptr, max_value_distance, 0);
    cv::setTrackbarPos("Distance", "PRACTICA_FINAL", init_value_distance);

    print_once = false;
  }


  value_choose_opt = cv::getTrackbarPos("Option", "PRACTICA_FINAL");
  value_distance = cv::getTrackbarPos("Distance", "PRACTICA_FINAL");

  out_image = in_image;

  switch(value_choose_opt) {

    case 0:
      //std::cout << "0: Original in cvMat and PCL\n" << std::endl;
      detect_person(out_image);
      //if (is_detected){
      //  std::cout << "Hay Persona\n" << std::endl;
      //}
      // if detect_person
      // make function
      //out_image = in_image;
      break;

    case 1:
      std::cout << "1: Detect person\n" << std::endl;
      //out_image = green_tags_dt(in_image, value_hough, false);

      break;

    case 2:
      std::cout << "2: Extras\n" << std::endl;
      //out_image = blue_balls_dt(in_image, false);

      break;

  }
    
  cv::imshow("PRACTICA_FINAL",out_image);

  return out_image;
}

/**
  TO-DO
*/
pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
{
  pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;
  out_pointcloud = in_pointcloud;
  return out_pointcloud;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  auto cv_node = std::make_shared<ComputerVisionSubscriber>();
  auto pcl_node = std::make_shared<PCLSubscriber>();
  exec.add_node(cv_node);
  exec.add_node(pcl_node);
  exec.spin();
  
  rclcpp::shutdown();
  return 0;
}