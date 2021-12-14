#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PointStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_msgs/Int16.h>

class TransparentDetection{
private:
  ros::Subscriber sub_img;
  ros::Subscriber sub_camera_info;
  ros::Subscriber sub_pocket;
  ros::Subscriber sub_flag;
  ros::Publisher pub_average;
  ros::Publisher pub_deviation;
  ros::Publisher pub_circle;

  void mainCb(const sensor_msgs::ImagePtr &main_img_msg);
  void infoCb(const sensor_msgs::CameraInfoConstPtr &camera_info_msg);
  void pocketCb(const sensor_msgs::PointCloud2ConstPtr &point_msg);
  void flagCb(const std_msgs::Int16 flag_i);

  std::vector<cv::Mat> gray_vector;

  sensor_msgs::CameraInfoConstPtr info_msg;
  image_geometry::PinholeCameraModel cam_model_;
  pcl::PointCloud<pcl::PointXYZ> cloud_pocket_points;

public:
  TransparentDetection();
  int flag = 0;
};

TransparentDetection::TransparentDetection(){
  ros::NodeHandle nh("~");
  //std::string camera_topic = "/camera/color/image_raw";
  std::string camera_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color";
  sub_img = nh.subscribe(camera_topic, 1, &TransparentDetection::mainCb,this);
  sub_camera_info = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/camera_info", 1, &TransparentDetection::infoCb,this);
  sub_pocket = nh.subscribe("/apu_identify_calendar_node/cloud_pocket_points", 1, &TransparentDetection::pocketCb,this);
  sub_flag = nh.subscribe("/identify_calendar_node/flag", 1, &TransparentDetection::flagCb,this);
  //if (!ros::topic::waitForMessage<sensor_msgs::Image>(camera_topic, ros::Duration(10.0))) {
  if (!ros::topic::waitForMessage<sensor_msgs::Image>(camera_topic)) {
    ROS_ERROR("timeout exceeded while waiting for message on topic %s", camera_topic.c_str());
    exit(EXIT_FAILURE);
  }

  pub_average = nh.advertise<sensor_msgs::Image>("average_img", 1);
  pub_deviation = nh.advertise<sensor_msgs::Image>("deviation_img", 1);
  pub_circle = nh.advertise<sensor_msgs::Image>("circle_img", 1);
}


//=========camera_info==========================================================
void TransparentDetection::infoCb(const sensor_msgs::CameraInfoConstPtr &camera_info_msg) {
  info_msg = camera_info_msg;
}
//=========pocket==========================================================
void TransparentDetection::pocketCb(const sensor_msgs::PointCloud2ConstPtr &point_msg) {
  pcl::fromROSMsg(*point_msg, cloud_pocket_points);
}
//=============flag============================================
void TransparentDetection::flagCb(const std_msgs::Int16 flag_i) {
  flag = flag_i.data;
}

//=========メインコールバック==============================================================
void TransparentDetection::mainCb(const sensor_msgs::ImagePtr &main_img_msg) {
  std::cout << "transparent detection callback start" << std::endl;

  //=====flag = 1のときデータ取得====================================================================
  if(flag == 1){

    //msg->img_ptr
    cv_bridge::CvImagePtr input_img_ptr;
    try {
      input_img_ptr = cv_bridge::toCvCopy(main_img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //to gray
    cv::Mat input_img,gray_img;
    input_img = input_img_ptr->image;
    cv::cvtColor(input_img, gray_img, cv::COLOR_BGR2GRAY);

    gray_vector.push_back(gray_img);

  }

  //=====flag = 2のとき計算====================================================================
  else if(flag == 2){

    //確認
    if(gray_vector.size() == 0){
      std::cout << "gray_vector size is zero " << std::endl;
      return;
    }

    //======位置合わせ==============================================================
    //-----基準画像------------------------------------------
    cv::Mat src1,src2;
    src2 = gray_vector[gray_vector->size() - 1]->image;

    //----特徴点検出--------------------------------
    cv::Ptr<cv::ORB>  orb = cv::ORB::create(500); // 特徴点検出アルゴリズムの選択(ORB)
    std::vector<cv::KeyPoint> key1, key2;  // 検出したキーポイントの格納先
    orb->detect(src2, key2);  // キーポイントの検出

    // ------特徴量計算------------------------------
    cv::Mat des1, des2;
    orb->compute(src2, key2, des2);

    // 特徴点マッチングアルゴリズムの選択
    cv::Ptr<cv::DescriptorMatcher> hamming = cv::DescriptorMatcher::create("BruteForce-Hamming");

    //----位置合わせ後の格納先
    std::vector<cv::Mat> registrated_vector;

    //-----------対象画像----------------------
    for(int k = 0; k < gray_vector->size(); k++){
      src1 = gray_vector[k]->image;

      // キーポイント
      orb->detect(src1, key1);  //特徴点
      orb->compute(src1, key1, des1); ///特徴量

      if(des1.cols <= 0 || des2.cols <= 0){
        std::cout << "key-point not found" << std::endl;
        return;
      }

      // 特徴点マッチング
      // # 特徴量記述des1とdes2のマッチングを行い、結果をmatchへ書き込む
      std::vector<cv::DMatch> match;
      hamming->match(des1, des2, match);
      if(match.size() < 5){
        std::cout << "match is not enough" << std::endl;
        return;
      }

      // 特徴量距離の小さい順にソートする（選択ソート）
      for (int i = 0; i < match.size() - 1; i++) {
        double min = match[i].distance;
        int n = i;
        for (int j = i + 1; j < match.size(); j++) {
          if (min > match[j].distance) {
            n = j;
            min = match[j].distance;
          }
        }
        std::swap(match[i], match[n]);
      }
      // 上位50点を残して、残りのマッチング結果を削除する。
      if(match.size() > 50){
        match.erase(match.begin() + 50, match.end());
      }

      //KeyPoint -> Point2d
      std::vector<cv::Point2f> match_point1;
      std::vector<cv::Point2f> match_point2;
      for (int i = 0; i < match.size(); i++) {
        match_point1.push_back(key1[match[i].queryIdx].pt);
        match_point2.push_back(key2[match[i].trainIdx].pt);
      }


      //-------変換行列を算出-------------------------------------------------
      cv::Mat affine,affine_img;
      afine = cv::estimateAffine2D(match_point1, match_point2);
      if(match.size() < 5 || afine.cols * afine.rows  == 0){
        std::cout << "can't calculate homography" << std::endl;
        return;
      }
      //-----アフィン変換------------------------------------------------------
      cv::warpAffine(src1, affine_img, affine, src1.size(), cv::INTER_LINEAR);

      registrated_vector.push_back(affine_img);
    }

    //========特徴マップを作成======================================================
    if(registrated_vector.size() <= 0){
      std::cout << "deviation_vector_size is zero" << std::endl;
      return;
    }
    //-------平均---------------
    cv::Mat average_img = registrated_vector[0].clone();
    for(int i = 1; i < registrated_vector.size(); i++){
      average_img += deviation_vector[i];
    }
    average_img = average_img / registrated_vector.size();

    //------手法1----------------
    cv::Mat feature_map_1 = registrated_vector[0].clone();

    for(int i = 0; i < average_img.cols; i++){
      for(int j = 0; j < average_img.rows; j++){
        double feature = 0.0;
        for(int k = 0; k < registrated_vector.size(); k++){
          double diff_f  = abs(average_img.at(j,i) - registrated_vector[k].at(j,i));
          if(feature < diff_f){
            feature = diff_f;
          }
        }

      }
    }

    //------手法2（標準偏差偏差）----------------

    cv::Mat feature_map_2 = registrated_vector[0].clone();
    feature_map_2 = (feature_map_2 - average_img);
    feature_map_2 = feature_map_2.mul(feature_map_2);

    for(int i = 1; i < registrated_vector.size(); i++){
      cv::Mat dev_img = (deviation_vector[i] - average_img);
      feature_map_2 += dev_img.mul(dev_img);
    }

    feature_map_2 = feature_map_2 / registrated_vector.size();








    //==========偏差を求める==========================================================
    //---------位置を中央の画像のキーポイントに合わせる---------------------------
    if(diff_vector.size() > 3){



      sensor_msgs::ImagePtr output_average_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", average_img).toImageMsg();
      pub_average.publish(output_average_msg);
      sensor_msgs::ImagePtr output_deviation_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", deviation_img).toImageMsg();
      output_deviation_msg->header.frame_id = "head_rgbd_sensor_rgb_frame";
      pub_deviation.publish(output_deviation_msg);
    }
  }
}

//=======main===================================================================
int main (int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "IdentifyCase");
  TransparentDetection TransparentDetection;

  ros::Rate rate(1);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  //ros::spin();

	return 0;
}
