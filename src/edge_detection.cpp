#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <sstream>

class EdgeDetection{
private:
  ros::Subscriber sub;
  ros::Publisher pub_img;
  void dimgCb(const sensor_msgs::ImagePtr &img_msg);
  int num = 0;

public:
  EdgeDetection();
};

EdgeDetection::EdgeDetection(){
  ros::NodeHandle nh("~");
  std::string realsense_topic = "/camera/color/image_raw";
  sub = nh.subscribe(realsense_topic, 1, &EdgeDetection::dimgCb,this);
  if (!ros::topic::waitForMessage<sensor_msgs::Image>(realsense_topic, ros::Duration(10.0))) {
    ROS_ERROR("timeout exceeded while waiting for message on topic %s", realsense_topic.c_str());
    exit(EXIT_FAILURE);
  }

  pub_img = nh.advertise<sensor_msgs::Image>("depth_image", 1);
}

//=========コールバック==============================================================
void EdgeDetection::dimgCb(const sensor_msgs::ImagePtr &img_msg) {
  std::cout << "callback_start" << std::endl;
  cv_bridge::CvImagePtr input_img_ptr;
  cv::Mat input_img;
  try {
    input_img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  input_img = input_img_ptr->image;

  cv::Mat sobel_img, laplacian_img, tmp_img;
  //gray
  cv::cvtColor(input_img, input_img, cv::COLOR_BGR2GRAY);

  //sobel
  cv::Sobel(input_img, tmp_img, CV_32F, 1, 1, 3, 1);
  cv::convertScaleAbs(tmp_img, sobel_img, 1, 0);

  //laplacian
  cv::Laplacian(input_img, tmp_img, CV_32F, 3);
  cv::convertScaleAbs(tmp_img, laplacian_img, 1, 0);

  // Canny
  //input_img.convertTo(input_img, CV_8UC1);
  cv::Mat canny_img;
  cv::Canny(input_img, canny_img, 50, 200);

  cv::imshow("gray",input_img);
  cv::imshow("sobel",sobel_img);
  cv::imshow("laplacian",laplacian_img);
  cv::imshow("canny",canny_img);

  pub_img.publish(input_img_ptr->toImageMsg());
}

//=======main===================================================================
int main (int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "IdentifyCase");
  EdgeDetection EdgeDetection;

  ros::Rate rate(1);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  //ros::spin();

	return 0;
}
