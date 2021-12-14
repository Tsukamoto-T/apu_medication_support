#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

class TransparentDetection{
private:
  ros::Subscriber sub_1;
	ros::Subscriber sub_2;
  ros::Publisher pub_pre;
	ros::Publisher pub_now;
  ros::Publisher pub_diff;
  ros::Publisher pub_average;
  ros::Publisher pub_deviation;

  void mainCb(const sensor_msgs::ImagePtr &img_msg);
	void inputCb(const sensor_msgs::ImagePtr &img_msg);
  cv_bridge::CvImagePtr pre_img_ptr;
  std::vector<cv::Mat> diff_vector;
  std::vector<std::vector<cv::KeyPoint>> key_vector;
  std::vector<cv::Mat> des_vector;

public:
  TransparentDetection();
};

TransparentDetection::TransparentDetection(){
  ros::NodeHandle nh("~");
  std::string camera_topic = "/camera/color/image_raw";
  //std::string camera_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color";
	std::string pre_topic = "/apu_transparent_detection_node/now_img";
  sub_1 = nh.subscribe(camera_topic, 1, &TransparentDetection::mainCb,this);
	sub_2 = nh.subscribe(pre_topic, 1, &TransparentDetection::inputCb,this);
  if (!ros::topic::waitForMessage<sensor_msgs::Image>(camera_topic, ros::Duration(10.0))) {
    ROS_ERROR("timeout exceeded while waiting for message on topic %s", camera_topic.c_str());
    exit(EXIT_FAILURE);
  }

  pub_pre = nh.advertise<sensor_msgs::Image>("pre_img", 1);
	pub_now = nh.advertise<sensor_msgs::Image>("now_img", 1);
  pub_diff = nh.advertise<sensor_msgs::Image>("diff_img", 1);
  pub_average = nh.advertise<sensor_msgs::Image>("average_img", 1);
  pub_deviation = nh.advertise<sensor_msgs::Image>("deviation_img", 1);
}

//=========インプットコールバック==========================================================
void TransparentDetection::inputCb(const sensor_msgs::ImagePtr &img_msg) {
	//std::cout << "input_callback_start" << std::endl;
  try {
    pre_img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

//=========メインコールバック==============================================================
void TransparentDetection::mainCb(const sensor_msgs::ImagePtr &img_msg) {
  std::cout << "main_callback_start" << std::endl;

  //msg->img_ptr
  cv_bridge::CvImagePtr input_img_ptr;
  try {
    input_img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //imgの確認
  if(input_img_ptr == nullptr){
  std::cout << "img_ptr is NULL " << std::endl;
    return;
  }

  //gray
  cv::Mat input_img,gray_img;
  input_img = input_img_ptr->image;
  //cv::resize(input_img, input_img, cv::Size(), 0.3, 0.3);
  cv::cvtColor(input_img, gray_img, cv::COLOR_BGR2GRAY);

  //Publish
  sensor_msgs::ImagePtr output_gray_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_img).toImageMsg();
  pub_now.publish(output_gray_msg);

  //pre_imgの確認
  if(pre_img_ptr == nullptr){
    std::cout << "pre_img_ptr is NULL " << std::endl;
    return;
  }
  pub_pre.publish(pre_img_ptr->toImageMsg());

  //======位置合わせ+差分計算==============================================================
  cv::Mat src1,src2;
  src1 = pre_img_ptr->image;
  src2 = gray_img;

  // 特徴点検出アルゴリズムの選択
  // # ORB::create(検出特徴点数, scale factor, ...)
  cv::Ptr<cv::ORB>  orb = cv::ORB::create(500);

  // 検出したキーポイント（特徴点）を格納する配列
  std::vector<cv::KeyPoint> key1, key2;

  // キーポイントの検出
  orb->detect(src1, key1);
  orb->detect(src2, key2);

  // 特徴量記述の計算
  cv::Mat des1, des2;
  orb->compute(src1, key1, des1);
  orb->compute(src2, key2, des2);

  if(des1.cols <= 0 || des2.cols <= 0){
    std::cout << "key-point not found" << std::endl;
    return;
  }

  // 特徴点マッチングアルゴリズムの選択
  cv::Ptr<cv::DescriptorMatcher> hamming = cv::DescriptorMatcher::create("BruteForce-Hamming");
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

  // 特徴点マッチングの結果を画像化する
  cv::Mat match_img;
  cv::drawMatches(src1, key1, src2, key2, match, match_img);

  // findHomography関数を用いて射影変換行列を算出
  cv::Mat homography,homo_img;
  homography = cv::findHomography(match_point1, match_point2, CV_RANSAC);
  if(match.size() < 5 || homography.cols * homography.rows  == 0){
    std::cout << "can't calculate homography" << std::endl;
    return;
  }
  cv::warpPerspective(src1, homo_img, homography, src1.size(), cv::INTER_LINEAR);

  //差分計算
  cv::Mat diff_img;
  cv::absdiff(homo_img, src2, diff_img);
  //std::cout << diff_img << std::endl;

  //Publish
  sensor_msgs::ImagePtr output_diff_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", diff_img).toImageMsg();
  pub_diff.publish(output_diff_msg);

  //差分格納
  diff_vector.push_back(diff_img);
  key_vector.push_back(key2);
  des_vector.push_back(des2);

  //==========偏差を求める==========================================================
  //---------位置を中央の画像のキーポイントに合わせる---------------------------
  if(diff_vector.size() > 3){

    // 特徴点マッチングアルゴリズムの選択
    cv::Ptr<cv::DescriptorMatcher> hamming_v = cv::DescriptorMatcher::create("BruteForce-Hamming");
    int end = key_vector.size() - 1;
    std::vector<cv::Mat> deviation_vector;

    for(int k = 0; k < key_vector.size(); k++){
      // 特徴点マッチング
      // # 特徴量記述desのマッチングを行い、結果をmatchへ書き込む
      std::vector<cv::DMatch> match_v;
      hamming_v->match(des_vector[k], des_vector[end], match_v);
      if(match_v.size() < 5){
        std::cout << "match is not enough" << std::endl;
        continue;
      }
      // 特徴量距離の小さい順にソートする（選択ソート）
      for (int i = 0; i < match_v.size() - 1; i++) {
        double min = match_v[i].distance;
        int n = i;
        for (int j = i + 1; j < match_v.size(); j++) {
          if (min > match_v[j].distance) {
            n = j;
            min = match_v[j].distance;
          }
        }
        std::swap(match_v[i], match_v[n]);
      }
      // 上位50点を残して、残りのマッチング結果を削除する。
      if(match_v.size() > 50){
        match_v.erase(match_v.begin() + 50, match_v.end());
      }

      //KeyPoint -> Point2d
      std::vector<cv::Point2f> match_point1_v;
      std::vector<cv::Point2f> match_point2_v;
      for (int i = 0; i < match_v.size(); i++) {
        match_point1_v.push_back(key_vector[k][match_v[i].queryIdx].pt);
        match_point2_v.push_back(key_vector[end][match_v[i].trainIdx].pt);
      }

      // findHomography関数を用いて射影変換行列を算出
      cv::Mat homography_v,homo_img_v;
      homography_v = cv::findHomography(match_point1_v, match_point2_v, CV_RANSAC);
      if(match_v.size() < 5 || homography_v.cols * homography_v.rows  == 0){
        std::cout << "can't calculate homography" << std::endl;
        continue;
      }
      cv::warpPerspective(diff_vector[k], homo_img_v, homography_v, diff_vector[k].size(), cv::INTER_LINEAR);

      deviation_vector.push_back(homo_img_v);
    }


    //-------------画像の偏差を求める-----------------------------------

    if(deviation_vector.size() <= 0){
      std::cout << "deviation_vector_size is zero" << std::endl;
      return;
    }

    cv::Mat average_img = deviation_vector[0].clone();
    for(int i = 1; i < deviation_vector.size(); i++){
      average_img += deviation_vector[i];
    }

    std::cout << "deviation_vector_size" << deviation_vector.size() << std::endl;

    average_img = average_img / deviation_vector.size();

    cv::Mat deviation_img = deviation_vector[0].clone();
    deviation_img = (deviation_img - average_img);
    deviation_img = deviation_img.mul(deviation_img);

    for(int i = 1; i < deviation_vector.size(); i++){
      cv::Mat dev_img = (deviation_vector[i] - average_img);
      deviation_img += dev_img.mul(dev_img);
    }

    deviation_img = deviation_img / deviation_vector.size();

    sensor_msgs::ImagePtr output_average_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", average_img).toImageMsg();
    pub_average.publish(output_average_msg);
    sensor_msgs::ImagePtr output_deviation_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", deviation_img).toImageMsg();
    pub_deviation.publish(output_deviation_msg);
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
