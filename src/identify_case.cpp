#include <string>
#include <stdio.h>
#include <ros/ros.h>

#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
#include <time.h>
#include <cstdlib>
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <geometry_msgs/PointStamped.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include<std_msgs/Float64.h>

typedef pcl::PointXYZRGB PointT;

class IdentifyCase{
private:
  ros::Subscriber sub;
  ros::Publisher pub_all;
  ros::Publisher pub_plane;
  ros::Publisher pub_rest;
  ros::Publisher pub_rest_removal;
  ros::Publisher pub_cluster_points;
  ros::Publisher pub_cluster_calendar;
  ros::Publisher pub_calendar;
  ros::Publisher pub_case;
  ros::Publisher pub_case_removal;
  ros::Publisher pub_case_filtered;
  //ros::Publisher pub_case_unfiltered;

  ros::Publisher pub_cluster_all;
  ros::Publisher pub_case_points;
  ros::Publisher pub_st_point;
  ros::Publisher pub_time_point;
  ros::Publisher pub_suction_point;

  ros::Publisher pub_dist;

  void CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

public:
  float threshold_calendar = 0.005;
  int calendar_number_neighbors = 20;
  float threshold_case = 0.005;
  int case_number_neighbors = 20;
  float cluster_tolerance = 0.005;
  int max_cluster_size = 20000;
  int min_cluster_size = 500;
  float case_cluster_tolerance = 0.005;
  int max_case_cluster_size = 20000;
  int min_case_cluster_size = 500;
  double st_x = 0.07;
  double st_y = 0.05;
  double st_z = 0.045;
  int set_week = 1;
  int set_time = 7;
  pcl::PointXYZ calendar_cent;

  IdentifyCase();
  void Output_pub(pcl::PointCloud<PointT>::Ptr cloud_ ,sensor_msgs::PointCloud2 msgs_,ros::Publisher pub_);
  void Center_case(pcl::PointCloud<PointT>::Ptr cloud_cluster_,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_case_points);
};

IdentifyCase::IdentifyCase(){
  ros::NodeHandle nh("~");
  nh.param<float>("threshold_calendar", threshold_calendar, threshold_calendar);
  nh.param<int>("calendar_number_neighbors",calendar_number_neighbors,calendar_number_neighbors);
  nh.param<float>("threshold_case", threshold_case, threshold_case);
  nh.param<int>("case_number_neighbors",case_number_neighbors,case_number_neighbors);
  nh.param<float>("cluster_tolerance",cluster_tolerance,cluster_tolerance);
  nh.param<int>("max_cluster_size",max_cluster_size,max_cluster_size);
  nh.param<int>("min_cluster_size",min_cluster_size,min_cluster_size);
  nh.param<float>("case_cluster_tolerance",case_cluster_tolerance,case_cluster_tolerance);
  nh.param<int>("max_case_cluster_size",max_case_cluster_size,max_case_cluster_size);
  nh.param<int>("min_case_cluster_size",min_case_cluster_size,min_case_cluster_size);
  nh.param<double>("st_x",st_x,st_x);
  nh.param<double>("st_y",st_y,st_y);
  nh.param<double>("st_z",st_z,st_z);
  nh.param<int>("set_week",set_week,set_week);
  nh.param<int>("set_time",set_time,set_time);

  std::string hsr_topic = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";
  sub = nh.subscribe(hsr_topic, 1, &IdentifyCase::CloudCb,this);
  if (!ros::topic::waitForMessage<sensor_msgs::PointCloud2>(hsr_topic, ros::Duration(10.0))) {
    ROS_ERROR("timeout exceeded while waiting for message on topic %s", hsr_topic.c_str());
    exit(EXIT_FAILURE);
  }

  pub_all = nh.advertise<sensor_msgs::PointCloud2>("cloud_all", 1);
  pub_plane = nh.advertise<sensor_msgs::PointCloud2>("cloud_plane", 1);
  pub_rest = nh.advertise<sensor_msgs::PointCloud2>("cloud_rest", 1);
  pub_rest_removal = nh.advertise<sensor_msgs::PointCloud2>("cloud_rest_removal", 1);
  pub_cluster_points = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_points", 1);
  pub_cluster_calendar = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_calendar", 1);
  pub_calendar = nh.advertise<sensor_msgs::PointCloud2>("cloud_calendar", 1);
  pub_case = nh.advertise<sensor_msgs::PointCloud2>("cloud_case", 1);
  pub_case_removal = nh.advertise<sensor_msgs::PointCloud2>("cloud_case_removal", 1);
  pub_case_filtered = nh.advertise<sensor_msgs::PointCloud2>("cloud_case_filtered", 1);
  //pub_case_unfiltered = nh.advertise<sensor_msgs::PointCloud2>("cloud_case_unfiltered", 1);

  pub_cluster_all = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_all", 1);
  pub_case_points = nh.advertise<sensor_msgs::PointCloud2>("cloud_case_points", 1);
  pub_st_point = nh.advertise<sensor_msgs::PointCloud2>("cloud_st_point", 1);
  pub_time_point = nh.advertise<sensor_msgs::PointCloud2>("cloud_time_point", 1);
  pub_suction_point = nh.advertise<sensor_msgs::PointCloud2>("cloud_suction_point", 1);
  pub_dist = nh.advertise<std_msgs::Float64>("point_dist", 1);


}

//========出力関数=========================================================================
void IdentifyCase::Output_pub(pcl::PointCloud<PointT>::Ptr cloud_,sensor_msgs::PointCloud2 msgs_,ros::Publisher pub_){
  if(cloud_->size() <= 0){
    ROS_INFO("Size (%d)",cloud_);
  }
  pcl::toROSMsg(*cloud_, msgs_);
  msgs_.header.frame_id = "head_rgbd_sensor_rgb_frame";
  pub_.publish(msgs_);
}

//=======把持位置を求めて出力する関数==============================================================
void IdentifyCase::Center_case(pcl::PointCloud<PointT>::Ptr cloud_cluster_,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_case_points){
  double cent_y = 100.0;
  for(size_t i = 0; i < cloud_cluster_->size(); i++){
    if (cent_y > cloud_cluster_->points[i].y) {
      cent_y = cloud_cluster_->points[i].y;
    }
  }
  cent_y +=  0.01;

  double cent_x = 0.0;
  double cent_i = 0;
  for(size_t i = 0; i < cloud_cluster_->size(); i++){
    if((cloud_cluster_->points[i].y < cent_y + 0.003) && (cloud_cluster_->points[i].y > cent_y - 0.003)){
      cent_x += cloud_cluster_->points[i].x;
      cent_i += 1.0;
    }
  }
  cent_x /= cent_i;

  //-----カレンダーの紐をクラスタと認識させない---
  if((0.185>fabs(calendar_cent.x - cent_x))&&(0.258>fabs(calendar_cent.y - cent_y))){
    double dist = 0.0;
    double dist_min = 100.0;
    int center_point = 0;
    for(size_t i = 0; i < cloud_cluster_->size(); i++){
      dist = sqrt(pow((cent_x - cloud_cluster_->points[i].x),2.0) + (pow((cent_y - cloud_cluster_->points[i].y),2.0)));
      if (dist_min > dist){
        dist_min = dist;
        center_point = (int)i;
      }
    }
    if(0 < cloud_cluster_->size()){
      pcl::PointXYZ cent_;
      cent_.x = cloud_cluster_->points[center_point].x;
      cent_.y = cloud_cluster_->points[center_point].y;
      cent_.z = cloud_cluster_->points[center_point].z;
      cloud_case_points->points.push_back(cent_);
    }
  }
}

//=========コールバック==============================================================
void IdentifyCase::CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  std::cout << "callback_start" << std::endl;
  pcl::PointCloud<PointT>::Ptr cloud_all(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_input_rm(new pcl::PointCloud<PointT>);
  std::vector<int> index_all;
  pcl::fromROSMsg(*cloud_msg, *cloud_all);
  pcl::removeNaNFromPointCloud(*cloud_all,*cloud_input_rm,index_all);

  //=======壁平面検出==============================================================
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);  //試行回数
  seg.setDistanceThreshold (threshold_calendar);  //距離の閾値

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_input_rm);
  seg.segment (*inliers, *coefficients);
  //=======壁平面除去=============================================================
  pcl::PointCloud<PointT>::Ptr cloud_rest(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud_input_rm); //入力データ
  extract.setIndices (inliers); //インデックスの入力
  extract.setNegative (false);  //内部の負のパラメータの値を返す（平面のみ出力）
  extract.filter (*cloud_plane);  //入力データ内の平面データ
  extract.setNegative (true);  //trueの場合入力インデックスを除くすべてのポイントを返す
  extract.filter (*cloud_rest);

  //=====外れ値処理=====================================================================
  pcl::PointCloud<PointT>::Ptr cloud_rest_removal(new pcl::PointCloud<PointT>);
  pcl::StatisticalOutlierRemoval<PointT> sor_plane;
  sor_plane.setInputCloud(cloud_rest);
  sor_plane.setMeanK(calendar_number_neighbors);
  sor_plane.setStddevMulThresh(1.0);
  sor_plane.filter(*cloud_rest_removal);

  //======カレンダー以外の点群を除去==================================================
  //------クラスタリング-------------------------------------------------------------
  pcl::search::KdTree<PointT>::Ptr cluster_tree (new pcl::search::KdTree<PointT>);
  cluster_tree->setInputCloud(cloud_rest_removal);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ece;
  ece.setClusterTolerance (cluster_tolerance);//探索する半径の設定
  ece.setMinClusterSize (min_cluster_size);//最小点の数を設定
  ece.setMaxClusterSize (max_cluster_size);//最大の点の数を設定
  ece.setSearchMethod (cluster_tree);//検索先のポインタを指定
  ece.setInputCloud (cloud_rest_removal);//点群を入力
  ece.extract (cluster_indices);//クラスター情報を出力

  int j = 0;
  double min_cluster_point = 100.0;
  pcl::PointCloud<PointT>::Ptr cloud_cluster_calendar (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_points(new pcl::PointCloud<pcl::PointXYZ>);

  for (std::vector<pcl::PointIndices>::const_iterator ite = cluster_indices.begin();ite != cluster_indices.end(); ++ite){
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

    for(std::vector<int>::const_iterator pite = ite->indices.begin(); pite != ite->indices.end(); pite++){
      cloud_cluster->points.push_back(cloud_rest_removal->points[*pite]);
    }

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    //----クラスタごと中心を求める---------------------------------------------------------
    if(0 < cloud_cluster->size()){
      pcl::PointXYZ cluster_cent;
      cluster_cent.x = 0.0;
      cluster_cent.y = 0.0;
      cluster_cent.z = 0.0;
      for(size_t i = 0; i < cloud_cluster->size(); i++){
        cluster_cent.x += cloud_cluster->points[i].x;
        cluster_cent.y += cloud_cluster->points[i].y;
        cluster_cent.z += cloud_cluster->points[i].z;
      }
      cluster_cent.x /= static_cast<int>(cloud_cluster->size());
      cluster_cent.y /= static_cast<int>(cloud_cluster->size());
      cluster_cent.z /= static_cast<int>(cloud_cluster->size());
      cloud_cluster_points->points.push_back(cluster_cent);
      //---（0,0)に最も近い点（クラスタ）をさがし、そのクラスタを薬カレンダーとみなす----------------
      double cluster_dist;
      cluster_dist = sqrt(pow(cluster_cent.x ,2) + pow(cluster_cent.y,2));
      if (min_cluster_point > cluster_dist){
        calendar_cent = cluster_cent;
        min_cluster_point = cluster_dist;
        *cloud_cluster_calendar = *cloud_cluster;
      }
    }

    j++;
  }
  //==========出力==================================================================
  sensor_msgs::PointCloud2 msgs_all;
  pcl::toROSMsg(*cloud_input_rm, msgs_all);
  pub_all.publish(msgs_all);

  sensor_msgs::PointCloud2 msgs_plane;
  pcl::toROSMsg(*cloud_plane, msgs_plane);
  pub_plane.publish(msgs_plane);

  sensor_msgs::PointCloud2 msgs_rest;
  pcl::toROSMsg(*cloud_rest, msgs_rest);
  pub_rest.publish(msgs_rest);

  sensor_msgs::PointCloud2 msgs_rest_removal;
  pcl::toROSMsg(*cloud_rest_removal, msgs_rest_removal);
  pub_rest_removal.publish(msgs_rest_removal);

  sensor_msgs::PointCloud2 msgs_cluster_calendar;
  IdentifyCase::Output_pub(cloud_cluster_calendar,msgs_cluster_calendar,pub_cluster_calendar);

  sensor_msgs::PointCloud2 msgs_cluster_points;
  pcl::toROSMsg(*cloud_cluster_points, msgs_cluster_points);
  msgs_cluster_points.header.frame_id = "head_rgbd_sensor_rgb_frame";
  pub_cluster_points.publish(msgs_cluster_points);

  //std::cout << "薬カレンダーサイズ：" << cloud_cluster_calendar->size() << std::endl;
  if (cloud_cluster_calendar->size() <= 0){
    std::cout << "薬カレンダーを見つけられませんでした" << std::endl;
    return;
  }

  //=======カレンダー平面を検出（ケースの検出）===========================================
  pcl::SACSegmentation<PointT> seg_2;
  pcl::PointIndices::Ptr inliers_2 (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_2 (new pcl::ModelCoefficients);
  seg_2.setOptimizeCoefficients (true);
  seg_2.setModelType (pcl::SACMODEL_PLANE);
  seg_2.setMethodType (pcl::SAC_RANSAC);
  seg_2.setMaxIterations (100);  //試行回数
  seg_2.setDistanceThreshold (threshold_case);  //距離の閾値
  seg_2.setInputCloud (cloud_cluster_calendar);
  seg_2.segment (*inliers_2, *coefficients_2);

  //カレンダー平面除去=================================================================
  pcl::PointCloud<PointT>::Ptr cloud_calendar(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_case(new pcl::PointCloud<PointT>);
  pcl::ExtractIndices<PointT> extract_2;
  extract_2.setInputCloud (cloud_cluster_calendar);
  extract_2.setIndices (inliers_2);
  extract_2.setNegative (false);
  extract_2.filter (*cloud_calendar);
  extract_2.setNegative (true);
  extract_2.filter (*cloud_case);

  //外れ値処理=====================================================================
  pcl::PointCloud<PointT>::Ptr cloud_case_removal(new pcl::PointCloud<PointT>);
  pcl::StatisticalOutlierRemoval<PointT> sor_plane_2;
  sor_plane_2.setInputCloud(cloud_case);
  sor_plane_2.setMeanK(case_number_neighbors);
  sor_plane_2.setStddevMulThresh(1.0);
  sor_plane_2.filter(*cloud_case_removal);

  //====カレンダー平面より奥にある点を削除================
  /*
  pcl::PointIndices::Ptr inliers_3 (new pcl::PointIndices);
  std::cout << *inliers_3 << std::endl;
  double sensor_dist = 0.0;
  int l = 0;
  for(size_t i = 0; i < cloud_case_removal->size(); i++){
    sensor_dist = coefficients_2->values[0]*cloud_case_removal->points[i].x + coefficients_2->values[1]*cloud_case_removal->points[i].y +coefficients_2->values[2]*cloud_case_removal->points[i].z + coefficients_2->values[3] / sqrt(pow(coefficients_2->values[0],2) + pow(coefficients_2->values[1],2) + pow(coefficients_2->values[2],2));
    if (sensor_dist < 0){
      //inliers_3->indices[l] = i;
      l++;
    }
  }
  pcl::PointCloud<PointT>::Ptr cloud_case_filtered(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_case_unfiltered(new pcl::PointCloud<PointT>);
  pcl::ExtractIndices<PointT> extract_3;
  extract_3.setInputCloud (cloud_case_removal);
  extract_3.setIndices (inliers_3);
  extract_3.setNegative (false);
  extract_3.filter (*cloud_case_filtered);
  extract_3.setNegative (true);
  extract_3.filter (*cloud_case_unfiltered);
  */

  pcl::PointCloud<PointT>::Ptr cloud_case_filtered(new pcl::PointCloud<PointT>);
  pcl::PassThrough<PointT> pass_z;
  pass_z.setInputCloud(cloud_case_removal);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(0.0,calendar_cent.z);
  pass_z.filter(*cloud_case_filtered);


  //================クラスタリング====================================================
  pcl::search::KdTree<PointT>::Ptr case_cluster_tree (new pcl::search::KdTree<PointT>);
  case_cluster_tree->setInputCloud(cloud_case_filtered);
  std::vector<pcl::PointIndices> case_cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (case_cluster_tolerance);//探索する半径の設定
  ec.setMinClusterSize (min_case_cluster_size);//最小点の数を設定
  ec.setMaxClusterSize (max_case_cluster_size);//最大の点の数を設定
  ec.setSearchMethod (case_cluster_tree);//検索先のポインタを指定
  ec.setInputCloud (cloud_case_filtered);//点群を入力
  ec.extract (case_cluster_indices);//クラスター情報を出力

  int k = 0;

  pcl::PointCloud<PointT>::Ptr cloud_cluster_all (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_case_points(new pcl::PointCloud<pcl::PointXYZ>);

  for (std::vector<pcl::PointIndices>::const_iterator it = case_cluster_indices.begin();it != case_cluster_indices.end(); ++it){
    pcl::PointCloud<PointT>::Ptr cloud_case_cluster (new pcl::PointCloud<PointT>);
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++){
      cloud_case_cluster->points.push_back(cloud_case_filtered->points[*pit]);
    }

    cloud_case_cluster->width = cloud_case_cluster->points.size();
    cloud_case_cluster->height = 1;
    cloud_case_cluster->is_dense = true;
    //std::cout << cloud_case_cluster->points.size() << std::endl;
    *cloud_cluster_all = (*cloud_cluster_all) + (*cloud_case_cluster);

    //========クラスタごとの把持位置を求める==================================================
    IdentifyCase::Center_case(cloud_case_cluster,cloud_case_points);
    k++;
  }
  std::cout << "薬ケースクラスタ数：" << cloud_case_points->size() << std::endl;

  //===カレンダーの左上の座標を求める=====================================================
  double min_sum_point = 0.0;
  int min_i;

  for(size_t i = 0; i < cloud_calendar->size(); ++i)
  {
    if(min_sum_point > (cloud_calendar->points[i].x + cloud_calendar->points[i].y)){
      min_sum_point = cloud_calendar->points[i].x + cloud_calendar->points[i].y;
      min_i=(int)i;
    }
  }

  //---出力----------------------------------------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_st_point(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ st_point;
  st_point.x = cloud_calendar->points[min_i].x;
  st_point.y = cloud_calendar->points[min_i].y;
  st_point.z = cloud_calendar->points[min_i].z;
  cloud_st_point->points.push_back(st_point);
  //std::cout<<"左上"<<point.x<<point.y<<point.z<<std::endl;

  //========曜日、時点(曜日：[0-6]日-土,時刻[0-23])に合うケース位置を計算======================
  time_t timer = time(NULL);
  struct tm *date = localtime(&timer);
  //printf("曜日：%2d 時刻:%2d\n",date->tm_wday,date->tm_hour);
  std::string week;
  std::string time;
  double interval_x = 0.08;
  double interval_y = 0.06;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_time_point(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ tm_point;
  switch (set_week){
    case 1:
    tm_point.y = st_point.y + st_y;
    week = "月曜日";
    break;
    case 2:
    tm_point.y = st_point.y + st_y + interval_y;
    week = "火曜日";
    break;
    case 3:
    tm_point.y = st_point.y + st_y + interval_y * 2;
    week = "水曜日";
    break;
    case 4:
    tm_point.y = st_point.y + st_y + interval_y * 3;
    week = "木曜日";
    break;
    case 5:
    tm_point.y = st_point.y + st_y + interval_y * 4;
    week = "金曜日";
    break;
    case 6:
    tm_point.y = st_point.y + st_y + interval_y * 5;
    week = "土曜日";
    break;
    case 0:
    tm_point.y = st_point.y + st_y + interval_y * 6;
    week = "日曜日";
    break;
  }
  //std::cout << week << std::endl;

  switch (set_time){
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    tm_point.x = st_point.x + st_x;
    time = "あさ";
    break;

    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    tm_point.x = st_point.x + st_x + interval_x;
    time = "ひる";
    break;

    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    tm_point.x = st_point.x + st_x + interval_x * 2;
    time = "よる";
    break;

    case 22:
    case 23:
    case 0:
    case 1:
    case 2:
    case 3:
    tm_point.x = st_point.x + st_x + interval_x * 3;
    time = "ねる前";
    break;
  }
  //std::cout << time << std::endl;
  tm_point.z = st_point.z - st_z;
  cloud_time_point->points.push_back(tm_point);

  //========計算した座標の近くにケース（把持位置）があるか確認================================
  double max_distance = 0.03;
  double distance = 0.0;
  int suction_i = -1;
  tf::Transform transform_;
  static tf::TransformBroadcaster tf_;

  for(size_t i = 0; i < cloud_case_points->size(); i++){
    distance = sqrt(pow((cloud_case_points->points[i].x - tm_point.x),2.0) + pow((cloud_case_points->points[i].y - tm_point.y),2.0) + pow((cloud_case_points->points[i].z - tm_point.z),2.0));
    if (distance < max_distance){
      suction_i = (int)i;
    }
  }
  if (suction_i >= 0){
    std::cout << week << time << "の薬を確認しました" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_suction_point(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ suction_point;
    suction_point.x = cloud_case_points->points[suction_i].x;
    suction_point.y = cloud_case_points->points[suction_i].y;
    suction_point.z = cloud_case_points->points[suction_i].z;
    cloud_suction_point->points.push_back(suction_point);

    //------壁から把持位置までの距離を計算---------------------------------------------
    double point_dist = 0.0;
    point_dist = fabs(coefficients->values[0]*suction_point.x + coefficients->values[1]*suction_point.y +coefficients->values[2]*suction_point.z + coefficients->values[3]) / sqrt(pow(coefficients->values[0],2) + pow(coefficients->values[1],2) + pow(coefficients->values[2],2));
    //std::cout << point_dist <<std::endl;
    std_msgs::Float64 msgs_dist;
    msgs_dist.data = point_dist;
    pub_dist.publish(msgs_dist);

    //------ｔｆをはる------------------------------------------------------------------)
    /*--tf----
    transform_.setOrigin(tf::Vector3(suction_point.x, suction_point.y, suction_point.z));
    transform_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tf_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "head_rgbd_sensor_rgb_frame", "suction_case_frame"));
    */
    //----tf2_ros-----
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.frame_id = "head_rgbd_sensor_rgb_frame";
    transformStamped.child_frame_id =  "suction_case_frame";
    transformStamped.transform.translation.x = suction_point.x;
    transformStamped.transform.translation.y = suction_point.y;
    transformStamped.transform.translation.z = suction_point.z;

    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    transformStamped.header.stamp = ros::Time::now();
    br.sendTransform(transformStamped);


    //--------出力----------------------------------------------------------------
    sensor_msgs::PointCloud2 msgs_suction_point;
    pcl::toROSMsg(*cloud_suction_point, msgs_suction_point);
    msgs_suction_point.header.frame_id = "head_rgbd_sensor_rgb_frame";
    pub_suction_point.publish(msgs_suction_point);
  }else{
    //std::cout << week << time << "の薬はありません" << std::endl;
  }

  //==========出力==================================================================
  sensor_msgs::PointCloud2 msgs_calendar;
  IdentifyCase::Output_pub(cloud_calendar,msgs_calendar,pub_calendar);

  sensor_msgs::PointCloud2 msgs_case;
  IdentifyCase::Output_pub(cloud_case,msgs_case,pub_case);

  sensor_msgs::PointCloud2 msgs_case_removal;
  IdentifyCase::Output_pub(cloud_case_removal,msgs_case_removal,pub_case_removal);

  sensor_msgs::PointCloud2 msgs_case_filtered;
  IdentifyCase::Output_pub(cloud_case_filtered,msgs_case_filtered,pub_case_filtered);

  /*
  sensor_msgs::PointCloud2 msgs_case_unfiltered;
  IdentifyCase::Output_pub(cloud_case_unfiltered,msgs_case_unfiltered,pub_case_unfiltered);
  */

  sensor_msgs::PointCloud2 msgs_cluster_all;
  IdentifyCase::Output_pub(cloud_cluster_all,msgs_cluster_all,pub_cluster_all);

  sensor_msgs::PointCloud2 msgs_case_points;
  pcl::toROSMsg(*cloud_case_points, msgs_case_points);
  msgs_case_points.header.frame_id = "head_rgbd_sensor_rgb_frame";
  pub_case_points.publish(msgs_case_points);

  sensor_msgs::PointCloud2 msgs_st_point;
  pcl::toROSMsg(*cloud_st_point, msgs_st_point);
  msgs_st_point.header.frame_id = "head_rgbd_sensor_rgb_frame";
  pub_st_point.publish(msgs_st_point);

  sensor_msgs::PointCloud2 msgs_time_point;
  pcl::toROSMsg(*cloud_time_point, msgs_time_point);
  msgs_time_point.header.frame_id = "head_rgbd_sensor_rgb_frame";
  pub_time_point.publish(msgs_time_point);

}

//=======main===================================================================
int main (int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "IdentifyCase");
  IdentifyCase IdentifyCase;

  ros::Rate rate(1);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  //ros::spin();

	return 0;
}
