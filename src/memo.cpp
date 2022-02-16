//=====左上の点を特定==========================================================
// Canny->エッジ検出
cv::Mat canny_img, hough_img;
cv::Canny(registrated_vector[registrated_vector.size()-1], canny_img, 50, 100);

cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/canny.png", canny_img);

std::vector<cv::Vec2f> lines;
cv::HoughLines(canny_img, lines, 1, CV_PI / 180.0, 100);

hough_img = registrated_vector[registrated_vector.size()-1].clone();
std::cout << lines.size() << std::endl;
for(int i = 0; i < lines.size(); i++){
  float rho = lines[i][0];
  float theta = lines[i][1];
  double a = cos(theta), b = sin(theta);
  double x0 = a*rho, y0 = b*rho;
  cv::Point pt1(cvRound(x0 + 1000*(-b)),cvRound(y0 + 1000*(a)));
  cv::Point pt2(cvRound(x0 - 1000*(-b)),cvRound(y0 - 1000*(a)));
  cv::line(hough_img, pt1, pt2, cv::Scalar(255), 1, 8);
}

cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/edge.png", hough_img);


// 特徴点マッチングの結果を画像化する
cv::Mat match_img;
cv::drawMatches(src1, key1, src2, key2, match, match_img);

// findHomography関数(透視変換)
homography = cv::findHomography(match_point1, match_point2, CV_RANSAC);
cv::warpPerspective(src1, homo_img, homography, src1.size(), cv::INTER_LINEAR);

//差分計算
cv::Mat diff_img;
cv::absdiff(affine_img, src2, diff_img);

//Publish
sensor_msgs::ImagePtr output_diff_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", diff_img).toImageMsg();
pub_diff.publish(output_diff_msg);

//---すべてのpresent_point出力----------------------
present_point.y = upper_left_point.y + y_base;
for(int i = 0; i < 7; i++){
  present_point.x = upper_left_point.x + x_base;
  for (int j = 0; j < 4; j++){
    cloud_present_point->points.push_back(present_point);
    present_point.x = present_point.x + x_interval;
  }
  present_point.y = present_point.y + y_interval;
}

//====pcl->image=======
/*
sensor_msgs::Image image_;
if ((cloud_msg->width * cloud_msg->height) == 0){
  return;
}

PointT point_test;
point_test.x=present_point.x;
point_test.y=present_point.y;
point_test.z=present_point.z;
point_test.r=0;
point_test.g=0;
point_test.b=0;

cloud_all->points.push_back(point_test);

try{
  pcl::toROSMsg (*cloud_all, image_);
}
catch (std::runtime_error e){
  ROS_ERROR_STREAM("Error in converting cloud to image message: " << e.what());
}

pub_image.publish (image_);
*/

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



  //=====フィルタリング=============================================================
  Mat blur_img,gaussian_img;
  blur(f_1_rect, blur_img, Size(3,3));
  GaussianBlur(f_1_rect, gaussian_img, Size(3,3),1.3);

  imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/nothing_f_2_blur.png", blur_img);
  imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/nothing_f_2_gaus.png", gaussian_img);



  //^^^^^^^^^^距離に基づく特徴決定^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  std::vector<int> renge_score;
  int max_score = 0;
  std::vector<int> max_iterator;
  auto mean = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
  auto var = (std::inner_product(vec.begin(), vec.end(), vec.begin(), 0.0) - mean * mean * vec.size() )/vec.size();

  float feature_threshold = var*2;

  for(int i = 0; i < vec.size(); i++){
    int rp = 0;
    for(int j = 0; j < vec.size(); j++){
      if(((vec[i] - feature_threshold) < vec[j]) && (vec[j] < (vec[i] + feature_threshold))){
        rp++;
      }
    }
    renge_score.push_back(rp);

    if(max_score < rp){
      max_iterator.clear();
      max_score = rp;
    }
    if(rp == max_score){
      max_iterator.push_back(i);
    }
  }

  float standard_score = 0;
  for(int i = 0; i < max_iterator.size(); i++){
    float st_score = 0;
    for(int j = 0; j < vec.size(); j++){
      if(((vec[max_iterator[i]] - feature_threshold) < vec[j]) && (vec[j] < (vec[max_iterator[i]] + feature_threshold))){
        st_score = st_score + vec[j];
      }
    }
    st_score = st_score / max_score;
    standard_score = standard_score + st_score;
  }
  standard_score = standard_score / max_iterator.size();

  float feature = 0.0;
  for(int i = 0; i < vec.size(); i++){
    if((vec[i] - standard_score) > feature){
      feature = vec[i] - standard_score;
    }
  }

  //ofs << vec[1] << vec[5] << vec[10] << vec[14] << std::endl;
  //ofs << "var:" << var <<" max_score:" << max_score << " max_iterator.size():" << max_iterator.size() << " standard_score:" << standard_score << " feature:" << feature << std::endl;

  return(feature);


//==============ソートによる特徴量決定=========
std::vector<float> st_diff;
int raito_v = floor(ratio*vec.size());
float mean, var;
std::vector<float> vector_mean;
std::vector<float> vector_var;

for(int i = 0; i < vec.size(); i++){
  for(int j = 0; j < vec.size(); j++){
    st_diff.push_back(abs(vec[i]-vec[j]));
  }
  std::sort(st_diff.begin(),st_diff.end());
  st_diff.erase(st_diff.end() - raito_v, st_diff.end());
  mean = std::accumulate(st_diff.begin(), st_diff.end(), 0.0) / st_diff.size();
  var = (std::inner_product(st_diff.begin(), st_diff.end(), st_diff.begin(), 0.0) - mean * mean * st_diff.size() )/st_diff.size();
  vector_mean.push_back(mean);
  vector_var.push_back(var);
}

float min_var = 255;
std::vector<int> ite_min;
for(int i = 0; i < vector_var.size(); i++){
  if(vector_var[i] < min_var){
    min_var = vector_var[i];
    ite_min.clear();
    ite_min.push_back(i);
  }else if(vector_var[i] == min_var){
    ite_min.push_back(i);
  }
}

float standard_score = 0.0;
for(int i = 0; i < ite_min.size(); i++){
  standard_score = standard_score + ite_min[i];
}
standard_score = standard_score / ite_min.size();

float feature = 0.0;
for(int i = 0; i < vec.size(); i++){
  if((vec[i] - standard_score) > feature){
    feature = vec[i] - standard_score;
  }
  //==========================================




  //=======Cmakelist.txt========================
  set(CMAKE_CXX_FLAGS "-fPIC -fpermissive")
  add_executable(discriminator src/discriminator.cpp)

  find_package(PythonLibs 2.7 REQUIRED NumPy)
  include_directories(${PYTHON_INCLUDE_DIRS})
  target_link_libraries(discriminator ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
