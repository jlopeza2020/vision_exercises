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



#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <vector>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>

#include "image_geometry/pinhole_camera_model.h"


#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"


using namespace std::chrono_literals;
//pcl global vars
pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud);
cv::Mat image_processing(const cv::Mat in_image);
geometry_msgs::msg::TransformStamped extrinsicbf2of; 

//image global vars
cv::Matx33f K; //intrinsic values 
cv::Matx34f extrinsic_matrixbf2of;
//cv::Mat depth_image;

int value_choose_opt;
int value_distance;
int key;
bool print_once = true;
bool detected = false;

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

      subscription_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/head_front_camera/rgb/camera_info", qos, std::bind(&ComputerVisionSubscriber::topic_callback_in_params, this, std::placeholders::_1));
    
      //subscription_depth_= this->create_subscription<sensor_msgs::msg::Image>(
      //"/head_front_camera/depth_registered/image_raw", qos, std::bind(&ComputerVisionSubscriber::topic_callback_depth, this, std::placeholders::_1));

      // transform listener inialization
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
      sensor_msgs::msg::Image out_image; // message to be sent
      img_bridge.toImageMsg(out_image); // from cv_bridge to sensor_msgs::Image

      // Publish the data
      publisher_ -> publish(out_image);

    }

    void topic_callback_in_params(const sensor_msgs::msg::CameraInfo::SharedPtr msg) const
    {           
      // create camera model
      image_geometry::PinholeCameraModel camera_model = image_geometry::PinholeCameraModel();
      camera_model.fromCameraInfo(msg);

      //Obtain intrinsic matrix
      K = camera_model.intrinsicMatrix();

    }

    /*void topic_callback_depth(const sensor_msgs::msg::Image::SharedPtr msg) const
    {     
    
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      }
      catch (cv_bridge::Exception &e)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
        return;
      }

      depth_image = cv_ptr->image;
    }*/

    void on_timer(){

      try {
        // goes from base_footprint to optical frame 
        extrinsicbf2of = tf_buffer_->lookupTransform("head_front_camera_rgb_optical_frame", "base_footprint", tf2::TimePointZero);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
        return;
      }

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_info_;
    //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

};


class PCLSubscriber : public rclcpp::Node
{
  public:
    PCLSubscriber()
    : Node("opencv_subscriber")
    {
      auto qos = rclcpp::QoS( rclcpp::QoSInitialization( RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5 ));
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

      subscription_3d_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/head_front_camera/depth_registered/points", qos, std::bind(&PCLSubscriber::topic_callback_3d, this, std::placeholders::_1));
    
      // transform listener inialization
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Call on_timer function every 500ms
      timer_ = this->create_wall_timer(500ms, std::bind(&PCLSubscriber::on_timer, this));
      
      publisher_3d_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "pcl_points", qos);
    }

  private:

    void topic_callback_3d(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {    
      
      // Convert to PCL data type
      pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
      pcl::fromROSMsg(*msg, point_cloud);     

      // PCL Processing
      pcl::PointCloud<pcl::PointXYZRGB> pcl_pointcloud = pcl_processing(point_cloud);
      
      // Convert to ROS data type
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(pcl_pointcloud, output);
      output.header = msg->header;

      // Publish the data
      publisher_3d_ -> publish(output);
    }

    void on_timer(){

      try {
        // goes from base_footprint to optical frame 
        extrinsicbf2of = tf_buffer_->lookupTransform("head_front_camera_rgb_optical_frame", "base_footprint", tf2::TimePointZero);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
        return;
      }

    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

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
  //if(classId == 0){
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
  //}
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

        if (classIdPoint.x == 0){
          classIds.push_back(classIdPoint.x);
          confidences.push_back((float)confidence);
          boxes.push_back(cv::Rect(left, top, width, height));
          detected = true;
        }
        //classIds.push_back(classIdPoint.x);
        //confidences.push_back((float)confidence);
        //boxes.push_back(cv::Rect(left, top, width, height));
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

  // Give the configuration and weight files for the model
  std::string modelConfiguration = "/home/juloau/Desktop/vision/ros2_computer_vision/src/computer_vision/src/cfg/yolov3.cfg";
  std::string modelWeights = "/home/juloau/Desktop/vision/ros2_computer_vision/src/computer_vision/src/cfg/yolov3.weights";

  // Load the network
  cv::dnn::Net net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);

  net.setPreferableBackend(cv::dnn::DNN_TARGET_CPU);

  cv::Mat frame, blob;

  frame = image;

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
}

void lines_from_3D_to_2D_image(cv::Mat out_image){

  if(value_distance >= 3){
    for(int i = 3; i <= value_distance; i++){

      cv::Mat point_req1 = (cv::Mat_<float>(4,1) << i, 1.4, 0.0, 1.0);
      cv::Mat point_req2 = (cv::Mat_<float>(4,1) << i, -1.4, 0.0, 1.0);

      cv::Mat res = K*extrinsic_matrixbf2of*point_req1;
      cv::Mat res2 = K*extrinsic_matrixbf2of*point_req2;

      cv::Point center(res.at<float>(0, 0)/abs(res.at<float>(2, 0)), res.at<float>(1, 0)/abs(res.at<float>(2, 0)));
      cv::circle(out_image,center, 3, cv::Scalar(0, 0, 255), 2); // draw the circle on the image

      cv::Point center2(res2.at<float>(0, 0)/abs(res.at<float>(2, 0)),res2.at<float>(1, 0)/abs(res2.at<float>(2, 0)));
      cv::circle(out_image,center2, 3, cv::Scalar(0, 0, 255), 2); // draw the circle on the image

      cv::line(out_image, center, center2, cv::Scalar(0, 0, 255), 2);

      cv::putText(out_image, std::to_string(i), center2, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);

    }
  }
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

  auto rotation = extrinsicbf2of.transform.rotation;
  
  tf2::Matrix3x3 mat(tf2::Quaternion{rotation.x, rotation.y, rotation.z, rotation.w});
  
  extrinsic_matrixbf2of = cv::Matx34f( mat[0][0], mat[0][1], mat[0][2], extrinsicbf2of.transform.translation.x,
                                  mat[1][0], mat[1][1], mat[1][2], extrinsicbf2of.transform.translation.y,
                                  mat[2][0], mat[2][1], mat[2][2], extrinsicbf2of.transform.translation.z);


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
      std::cout << "0: Original in cvMat and PCL\n" << std::endl;
      
      // if detect_person
      // make function
      //out_image = in_image;
      
      break;

    case 1:
      //std::cout << "1: Detect person\n" << std::endl;
      detect_person(out_image);
      if (detected){
        std::cout << "Hay Persona\n" << std::endl;
        lines_from_3D_to_2D_image(out_image);


      }
      // if detect_person
      // make function
      //out_image = in_image;
      
      //out_image = green_tags_dt(in_image, value_hough, false);

      break;

    case 2:
      std::cout << "2: Extras\n" << std::endl;
      //out_image = blue_balls_dt(in_image, false);
       //out_image = in_image;
      
      break;
    detected = false;

  }
    
  cv::imshow("PRACTICA_FINAL",out_image);

  return out_image;
}

/**
  TO-DO
*/
void print_cubes(pcl::PointCloud<pcl::PointXYZRGB>& cloud, float x_center, float y_center,float z_center, int r, int g, int b){

  float dim = 0.15;
  float step = 0.008;
  
  for(float i = 0.0; i < dim; i+= step){
    for(float j = 0.0; j < dim; j+= step){
      for(float k = 0.0; k < dim; k+= step){

        pcl::PointXYZRGB point;
        point.x = x_center + i;
        point.y = y_center + j;
        point.z = z_center + k;
        point.r = r;
        point.g = g;
        point.b = b;
        cloud.push_back(point);

      }
    }
  }
}

void lines_from_3D_to_2D_pcl(pcl::PointCloud<pcl::PointXYZRGB>& cloud){

  
  int r = 255; 
  int g = 0; 
  int b = 0;

  if (value_distance >= 3){

    for(int i = 3; i <= value_distance; i++){

      //Left point
      geometry_msgs::msg::PointStamped point_in_left;
      point_in_left.header.frame_id = "base_footprint";
      point_in_left.point.x = i;
      point_in_left.point.y = 1.4;
      point_in_left.point.z = 0;

      geometry_msgs::msg::PointStamped point_out_left;
      point_out_left.header.frame_id = "head_front_camera_rgb_optical_frame";

      // Transform the point from base footprint to optical frame
      tf2::doTransform(point_in_left, point_out_left, extrinsicbf2of);

      print_cubes(cloud,point_out_left.point.x, point_out_left.point.y, point_out_left.point.z, r, g, b);

      //Right point
      geometry_msgs::msg::PointStamped point_in_right;
      point_in_right.header.frame_id = "base_footprint";
      point_in_right.point.x = i;
      point_in_right.point.y = -1.4;
      point_in_right.point.z = 0;

      geometry_msgs::msg::PointStamped point_out_right;
      point_out_right.header.frame_id = "head_front_camera_rgb_optical_frame";

      // Transform the point from base footprint to optical frame
      tf2::doTransform(point_in_right, point_out_right, extrinsicbf2of);

      print_cubes(cloud, point_out_right.point.x, point_out_right.point.y, point_out_right.point.z, r, g, b);
      r -= 40;
      g += 40;
      b += 40;

    }
  }
}


pcl::PointCloud<pcl::PointXYZRGB> pcl_processing(const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
{
  pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;
  out_pointcloud = in_pointcloud;


  switch(value_choose_opt) {

    case 0:
      //out_pointcloud = in_pointcloud;
      // if detect_person
      // make function
      
      break;

    case 1:
      //std::cout << "1: Detect person\n" << std::endl;
      
      //out_pointcloud = in_pointcloud;
      if (detected){
        std::cout << "Hay Persona\n" << std::endl;
        lines_from_3D_to_2D_pcl(out_pointcloud);

      }
      // if detect_person
      // make function
      //out_pointcloud = in_pointcloud;
      
      //out_image = green_tags_dt(in_image, value_hough, false);

      break;

    case 2:
       //out_pointcloud = in_pointcloud;
      
      break;

  }
    

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