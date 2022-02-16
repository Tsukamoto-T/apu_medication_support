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
#include <std_msgs/Float32MultiArray.h>
#include <numeric>

//std::ofstream ofs("/home/stl/catkin_ws/src/apu_medication_support/data/data.txt");
class TransparentDetection{
private:
  ros::Subscriber sub_img;
  ros::Subscriber sub_camera_info;
  ros::Subscriber sub_pocket;
  ros::Subscriber sub_flag;
  ros::Publisher pub_feature_map;
  ros::Publisher pub_feature;
  ros::Publisher pub_flag;

  void imgCb(const sensor_msgs::ImagePtr &img_msg);
  void infoCb(const sensor_msgs::CameraInfoConstPtr &camera_info_msg);
  void pocketCb(const sensor_msgs::PointCloud2ConstPtr &point_msg);
  void flagCb(const std_msgs::Int16 flag_i);

  float computeFeature(std::vector<float> vec);

  std::vector<cv::Mat> gray_vector;
  sensor_msgs::CameraInfoConstPtr info_msg;
  image_geometry::PinholeCameraModel cam_model_;
  pcl::PointCloud<pcl::PointXYZ> cloud_pocket_points;

public:
  TransparentDetection();
  int flag = 0;
  float ratio = 0.05;
};

TransparentDetection::TransparentDetection(){
  ros::NodeHandle nh("~");
  nh.param<float>("ratio",ratio,ratio);

  //std::string camera_topic = "/camera/color/image_raw";
  std::string camera_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color";
  sub_img = nh.subscribe(camera_topic, 1, &TransparentDetection::imgCb,this);
  sub_camera_info = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/camera_info", 1, &TransparentDetection::infoCb,this);
  sub_pocket = nh.subscribe("/apu_identify_calendar_node/cloud_target_points", 1, &TransparentDetection::pocketCb,this);
  sub_flag = nh.subscribe("/apu_identify_calendar_node/flag", 1, &TransparentDetection::flagCb,this);
  //if (!ros::topic::waitForMessage<sensor_msgs::Image>(camera_topic, ros::Duration(10.0))) {
  //if (!ros::topic::waitForMessage<sensor_msgs::Image>(camera_topic)) {
  //  ROS_ERROR("timeout exceeded while waiting for message on topic %s", camera_topic.c_str());
  //  exit(EXIT_FAILURE);
  //}

  pub_feature_map = nh.advertise<sensor_msgs::Image>("feature_map", 1);
  pub_feature = nh.advertise<std_msgs::Float32MultiArray>("feature", 1);
  pub_flag = nh.advertise<std_msgs::Int16>("/apu_identify_calendar_node/flag", 1);

}


//========computeFeature============================================================
float TransparentDetection::computeFeature(std::vector<float> vec){
  //------中央値と最大値による特徴量決定---------------------------------------

  std::sort(vec.begin(),vec.end());
  int median = std::floor(vec.size()/2);
  float feature;
  feature = vec.back()-vec[median];

  return(feature);

}


//=========camera_info==========================================================
void TransparentDetection::infoCb(const sensor_msgs::CameraInfoConstPtr &camera_info_msg) {
  info_msg = camera_info_msg;
}
//=========pocket==========================================================
void TransparentDetection::pocketCb(const sensor_msgs::PointCloud2ConstPtr &point_msg) {
  pcl::fromROSMsg(*point_msg, cloud_pocket_points);

  //=====flag = 2のとき計算====================================================================
  if(flag == 2){
    std::cout << "sub target point" << std::endl;
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

      // 特徴点マッチングの結果を画像化する
      cv::Mat match_img;
      cv::drawMatches(src1, key1, src2, key2, match, match_img);
      std::ostringstream oss_;
      oss_  << std::setw(3) << k;
      cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/match_img_"+ oss_.str() +".png", match_img);


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

    for(int i = 0; i < registrated_vector.size(); i++){
      registrated_vector[i].convertTo(registrated_vector[i],CV_32F,1.0/255);
    }

    //------距離に基づく基準値決定(computeFeature)---------------------------------------
    std::vector<float> vector_data;
    cv::Mat feature_map = cv::Mat::zeros(registrated_vector[0].size(),CV_32F);
    float standard = 0.0;

    for(int i = 0; i < feature_map.cols; i++){
      for(int j = 0; j < feature_map.rows; j++){
        for(int k = 0; k < registrated_vector.size(); k++){
          vector_data.push_back(registrated_vector[k].at<float>(j,i));
        }
        feature_map.at<float>(j,i) = TransparentDetection::computeFeature(vector_data);
        vector_data.clear();
      }
    }

    cv::Mat feature_map_cv(registrated_vector[0].size(),CV_8U);
    feature_map.convertTo(feature_map_cv,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/feature_map.png", feature_map_cv);


    //======時点のみ切り出し=========================================================
    if(info_msg == nullptr ){
      std::cout << "NULL " << std::endl;
      return;
    }

    cam_model_.fromCameraInfo(info_msg);
    std::vector<cv::Point2d> pre_rectangle(2);
    for(int i = 0; i < cloud_pocket_points.size(); i++){
      //std::cout << "i " << i << std::endl;
      //std::cout << "pre3d : x " << cloud_pocket_points.points[i].x << " y " << cloud_pocket_points.points[i].y << " z " << cloud_pocket_points.points[i].z << std::endl;
      cv::Point3d point_3d(cloud_pocket_points.points[i].x, cloud_pocket_points.points[i].y, cloud_pocket_points.points[i].z);
      cv::Point2d uv = cam_model_.project3dToPixel(point_3d);
      //std::cout << "pre2d : u " << uv.x << " v " << uv.y << std::endl;
      pre_rectangle[i] = uv;
    }

    //時点の切り出し
    cv::Mat target_map;
    target_map = cv::Mat(feature_map, cv::Rect(pre_rectangle[0],pre_rectangle[1]));

    cv::Mat target_map_cv;
    target_map.convertTo(target_map_cv,CV_8U,255);
    cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/target.png", target_map_cv);

    //-----出力----------------------------------------------
    //sensor_msgs::ImagePtr feature_map_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", feature_map).toImageMsg();
    //feature_map_msg->header = header;
    //pub_feature_map.publish(feature_map_msg);
    //sensor_msgs::ImagePtr feature_map_2_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", feature_map_2).toImageMsg();
    //feature_map_2_msg->header = header;
    //pub_feature_map_2.publish(feature_map_2_msg);

    //=====ヒストグラム============================================================
    // 分割レベル
    int binNum = 256;
    int histSize[] = {binNum};

    float value_ranges[] = { 0, 256 };
    const float* ranges[] = { value_ranges };
    cv::MatND hist;
    // 0 番目のチャンネルからヒストグラムを求めます．
    int channels[] = {0};
    int dims = 1; // 入力行列（画像）のチャンネル数

    calcHist( &target_map_cv, 1, channels, cv::Mat(), // マスクは利用しません
      hist, dims, histSize, ranges,
      true, // ヒストグラムは一様です
      false );
    double maxVal=0;
    minMaxLoc(hist, 0, &maxVal, 0, 0);

    //---ヒストグラム書き出し------
    std::ofstream ofs("/home/stl/catkin_ws/src/apu_medication_support/data/histgram.csv");
    for( int binId = 0; binId < binNum; binId++ ){
      float binVal = hist.at<float>(binId);
      ofs << binId << "\t" << binVal << std::endl;
    }

    //---target_map書き出し------
    std::ofstream ofs_2("/home/stl/catkin_ws/src/apu_medication_support/data/target_map.csv");

    //---target_mapデータpub------
    std_msgs::Float32MultiArray feature_array;
    int f_num = 0;
    feature_array.data.resize(target_map.cols*target_map.rows);
    for(int i = 0; i < target_map.cols; i++){
      for(int j = 0; j < target_map.rows; j++){
        feature_array.data[f_num] = target_map.at<float>(j,i);
        ofs_2 << target_map.at<float>(j,i) << std::endl;
        f_num++;
      }
    }
    pub_feature.publish(feature_array);


    //-----確認用画像出力--------------------------------------------
    cv::Mat img;
    for (int i = 0; i < registrated_vector.size(); i++) {
      registrated_vector[i].convertTo(img,CV_8U,255);
      //取得した画像を連番画像で保存
      std::ostringstream oss;
      oss << std::setfill('0') << std::setw(3) << i;
      cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/registrated_image_" + oss.str() + ".png", img);
    }

    std_msgs::Int16 flag_num;
    flag_num.data = 3;
    pub_flag.publish(flag_num);
  }
}
//=============flag============================================
void TransparentDetection::flagCb(const std_msgs::Int16 flag_i) {
  flag = flag_i.data;
  std::cout << "flag " << flag << std::endl;
}


//=========イメージコールバック==============================================================
void TransparentDetection::imgCb(const sensor_msgs::ImagePtr &img_msg) {

  //=====flag = 1のときデータ取得====================================================================
  if(flag == 1){
    std::cout << "getting data" << std::endl;

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

  ros::Rate rate(2);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  //ros::spin();

	return 0;
}
