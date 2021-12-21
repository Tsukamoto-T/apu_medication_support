#include <ros/ros.h>
#include <fstream>
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
  ros::Publisher pub_feature_map_1;
  ros::Publisher pub_feature_map_2;

  void imgCb(const sensor_msgs::ImagePtr &img_msg);
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
  sub_img = nh.subscribe(camera_topic, 1, &TransparentDetection::imgCb,this);
  sub_camera_info = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/camera_info", 1, &TransparentDetection::infoCb,this);
  sub_pocket = nh.subscribe("/apu_identify_calendar_node/cloud_pocket_points", 1, &TransparentDetection::pocketCb,this);
  sub_flag = nh.subscribe("/apu_identify_calendar_node/flag", 1, &TransparentDetection::flagCb,this);
  //if (!ros::topic::waitForMessage<sensor_msgs::Image>(camera_topic, ros::Duration(10.0))) {
  //if (!ros::topic::waitForMessage<sensor_msgs::Image>(camera_topic)) {
  //  ROS_ERROR("timeout exceeded while waiting for message on topic %s", camera_topic.c_str());
  //  exit(EXIT_FAILURE);
  //}

  pub_feature_map_1 = nh.advertise<sensor_msgs::Image>("feature_map_1", 1);
  pub_feature_map_1 = nh.advertise<sensor_msgs::Image>("feature_map_2", 1);
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

  //=====flag = 2のとき計算====================================================================
  if(flag == 2){
    //確認
    if(gray_vector.size() == 0){
      std::cout << "gray_vector size is zero " << std::endl;
      return;
    }

    //======位置合わせ==============================================================
    //-----基準画像------------------------------------------
    cv::Mat src1,src2;
    src2 = gray_vector[gray_vector.size() - 1];

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
    for(int k = 0; k < gray_vector.size(); k++){
      src1 = gray_vector[k];

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
      affine = cv::estimateAffine2D(match_point1, match_point2);
      if(match.size() < 5 || affine.cols * affine.rows  == 0){
        std::cout << "can't calculate homography" << std::endl;
        return;
      }
      //-----アフィン変換------------------------------------------------------
      cv::warpAffine(src1, affine_img, affine, src1.size(), cv::INTER_LINEAR);

      registrated_vector.push_back(affine_img);
    }



    //========特徴マップを作成======================================================
    if(registrated_vector.size() <= 0){
      std::cout << "registrated_vector_size is zero" << std::endl;
      return;
    }

    //-------平均---------------
    cv::Mat average_img = cv::Mat::zeros(registrated_vector[0].size(),CV_32F);
    //std::cout << registrated_vector[0].at<float>(100,100) << std::endl;
    for(int i = 0; i < registrated_vector.size(); i++){
      registrated_vector[i].convertTo(registrated_vector[i],CV_32F,1.0/255);
      average_img = average_img + registrated_vector[i];
    }
    average_img = average_img / registrated_vector.size();

    //------手法1----------------
    cv::Mat feature_map_1 = cv::Mat::zeros(registrated_vector[0].size(),CV_32F);
    for(int i = 0; i < feature_map_1.cols; i++){
      for(int j = 0; j < feature_map_1.rows; j++){
        double feature = 0.0;
        for(int k = 0; k < registrated_vector.size(); k++){
          double diff_f  = abs(average_img.at<float>(j,i) - registrated_vector[k].at<float>(j,i));
          if(feature < diff_f){
            feature = diff_f;
          }
        }
        feature_map_1.at<float>(j,i) = feature;
      }
    }

    //------手法2（標準偏差偏差）----------------

    cv::Mat feature_map_2 = registrated_vector[0].clone();
    feature_map_2 = (feature_map_2 - average_img);
    feature_map_2 = feature_map_2.mul(feature_map_2);

    for(int i = 1; i < registrated_vector.size(); i++){
      cv::Mat dev_img = (registrated_vector[i] - average_img);
      feature_map_2 += dev_img.mul(dev_img);
    }

    feature_map_2 = feature_map_2 / registrated_vector.size();

    double max_f;
    cv::minMaxLoc(feature_map_2,NULL,&max_f);
    feature_map_2 = feature_map_2 * (1 / max_f);

    cv::Mat write_img(registrated_vector[0].size(),CV_8U);
    average_img.convertTo(write_img,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/average.png", write_img);
    feature_map_1.convertTo(write_img,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/feature_map_1.png", write_img);
    feature_map_2.convertTo(write_img,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/feature_map_2.png", write_img);

    //======時点のみ切り出し=========================================================
    if(info_msg == nullptr ){
      std::cout << "NULL " << std::endl;
      return;
    }

    cam_model_.fromCameraInfo(info_msg);
    std::vector<cv::Point2d> pre_rectangle(8);
    for(int i = 0; i < cloud_pocket_points.size(); i++){
      std::cout << "i " << i << std::endl;
      std::cout << "pre3d : x " << cloud_pocket_points.points[i].x << " y " << cloud_pocket_points.points[i].y << " z " << cloud_pocket_points.points[i].z << std::endl;
      cv::Point3d point_3d(cloud_pocket_points.points[i].x, cloud_pocket_points.points[i].y, cloud_pocket_points.points[i].z);
      cv::Point2d uv = cam_model_.project3dToPixel(point_3d);
      std::cout << "pre2d : u " << uv.x << " v " << uv.y << std::endl;
      pre_rectangle[i] = uv;
    }

    //時点を矩形で囲む
    cv::Mat f_1 = feature_map_1.clone();
    cv::rectangle(f_1,pre_rectangle[0],pre_rectangle[1], cv::Scalar(1), 1);
    cv::rectangle(f_1,pre_rectangle[2],pre_rectangle[3], cv::Scalar(1), 1);
    cv::rectangle(f_1,pre_rectangle[4],pre_rectangle[5], cv::Scalar(1), 1);
    cv::rectangle(f_1,pre_rectangle[6],pre_rectangle[7], cv::Scalar(1), 1);
    f_1.convertTo(write_img,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/rectangle.png", write_img);

    //時点の切り出し
    cv::Mat f_dst_1_1,f_dst_1_2,f_dst_1_3,f_dst_1_4;
    f_dst_1_1 = cv::Mat(feature_map_1, cv::Rect(pre_rectangle[0],pre_rectangle[1]));
    f_dst_1_2 = cv::Mat(feature_map_1, cv::Rect(pre_rectangle[2],pre_rectangle[3]));
    f_dst_1_3 = cv::Mat(feature_map_1, cv::Rect(pre_rectangle[4],pre_rectangle[5]));
    f_dst_1_4 = cv::Mat(feature_map_1, cv::Rect(pre_rectangle[6],pre_rectangle[7]));

    cv::Mat f_dst_2_1,f_dst_2_2,f_dst_2_3,f_dst_2_4;
    f_dst_2_1 = cv::Mat(feature_map_2, cv::Rect(pre_rectangle[0],pre_rectangle[1]));
    f_dst_2_2 = cv::Mat(feature_map_2, cv::Rect(pre_rectangle[2],pre_rectangle[3]));
    f_dst_2_3 = cv::Mat(feature_map_2, cv::Rect(pre_rectangle[4],pre_rectangle[5]));
    f_dst_2_4 = cv::Mat(feature_map_2, cv::Rect(pre_rectangle[6],pre_rectangle[7]));

    f_dst_1_1.convertTo(write_img,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/feature_map_1_1.png", write_img);
    f_dst_1_2.convertTo(write_img,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/feature_map_1_2.png", write_img);
    f_dst_1_3.convertTo(write_img,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/feature_map_1_3.png", write_img);
    f_dst_1_4.convertTo(write_img,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/feature_map_1_4.png", write_img);

    f_dst_2_1.convertTo(write_img,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/feature_map_2_1.png", write_img);
    f_dst_2_2.convertTo(write_img,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/feature_map_2_2.png", write_img);
    f_dst_2_3.convertTo(write_img,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/feature_map_2_3.png", write_img);
    f_dst_2_4.convertTo(write_img,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/feature_map_2_4.png", write_img);

    std::ofstream writing_file;
    std::string filename = "/home/stl/catkin_ws/src/apu_medication_support/data/feature.txt";
    writing_file.open(filename, std::ios::out);
    writing_file << "feature1_1" << std::endl;
    writing_file << f_dst_1_1 << std::endl;
    writing_file << "--------------------------------------------------------------------------------" << std::endl;
    writing_file << "feature1_2" << std::endl;
    writing_file << f_dst_1_2 << std::endl;
    writing_file << "--------------------------------------------------------------------------------" << std::endl;
    writing_file << "feature1_3" << std::endl;
    writing_file << f_dst_1_3 << std::endl;
    writing_file << "--------------------------------------------------------------------------------" << std::endl;
    writing_file << "feature1_4" << std::endl;
    writing_file << f_dst_1_4 << std::endl;
    writing_file << "--------------------------------------------------------------------------------" << std::endl;
    writing_file << "--------------------------------------------------------------------------------" << std::endl;
    writing_file << "feature2_1" << std::endl;
    writing_file << f_dst_2_1 << std::endl;
    writing_file << "--------------------------------------------------------------------------------" << std::endl;
    writing_file << "feature2_2" << std::endl;
    writing_file << f_dst_2_2 << std::endl;
    writing_file << "--------------------------------------------------------------------------------" << std::endl;
    writing_file << "feature2_3" << std::endl;
    writing_file << f_dst_2_3 << std::endl;
    writing_file << "--------------------------------------------------------------------------------" << std::endl;
    writing_file << "feature2_4" << std::endl;
    writing_file << f_dst_2_4 << std::endl;
    writing_file.close();

    //-----出力----------------------------------------------
    //sensor_msgs::ImagePtr feature_map_1_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", feature_map_1).toImageMsg();
    //feature_map_1_msg->header = header;
    //pub_feature_map_1.publish(feature_map_1_msg);
    //sensor_msgs::ImagePtr feature_map_2_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", feature_map_2).toImageMsg();
    //feature_map_2_msg->header = header;
    //pub_feature_map_2.publish(feature_map_2_msg);

    //-----確認用画像出力--------------------------------------------
    cv::Mat img;
    for (int i = 0; i < registrated_vector.size(); i++) {
      registrated_vector[i].convertTo(img,CV_8U,255);
      //取得した画像を連番画像で保存
      std::ostringstream oss;
      oss << std::setfill('0') << std::setw(3) << i;
      cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/registrated_image_" + oss.str() + ".png", img);
    }
    flag = 3;
  }
}


//=========イメージコールバック==============================================================
void TransparentDetection::imgCb(const sensor_msgs::ImagePtr &img_msg) {
  std::cout << "transparent detection callback start" << std::endl;

  //=====flag = 1のときデータ取得====================================================================
  if(flag == 1){

    //msg->img_ptr
    cv_bridge::CvImagePtr input_img_ptr;
    try {
      input_img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //to gray
    cv::Mat input_img,gray_img,convert_img;
    input_img = input_img_ptr->image;
    cv::cvtColor(input_img, gray_img, cv::COLOR_BGR2GRAY);
    gray_vector.push_back(gray_img);
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
