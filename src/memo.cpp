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
