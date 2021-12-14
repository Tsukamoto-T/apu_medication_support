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



//======時点のみ切り出し=========================================================
if(info_msg == nullptr ){
  std::cout << "NULL " << std::endl;
  return;
}

cam_model_.fromCameraInfo(info_msg);
std::vector<cv::Point2d> pre_rectangle(2);

for(int i = 0; i < cloud_pocket_points.size() ;i++){
  std::cout << "i " << i << std::endl;
  std::cout << "pre3d : x " << cloud_pocket_points.points[i].x << " y " << cloud_pocket_points.points[i].y << " z " << cloud_pocket_points.points[i].z << std::endl;
  cv::Point3d point_3d(cloud_pocket_points.points[i].x, cloud_pocket_points.points[i].y, cloud_pocket_points.points[i].z);
  cv::Point2d uv = cam_model_.project3dToPixel(point_3d);
  std::cout << "pre2d : u " << uv.x << " v " << uv.y << std::endl;
  std::cout << "imagesize " << input_img.size() << std::endl;
  pre_rectangle[i] = uv;
}

std::cout << pre_rectangle[0] << std::endl;
std::cout << pre_rectangle[1] << std::endl;

cv::rectangle(input_img,pre_rectangle[0],pre_rectangle[1], cv::Scalar(255,0,0), 5);

//Publish
sensor_msgs::ImagePtr circle_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", input_img).toImageMsg();
pub_circle.publish(circle_msg);
