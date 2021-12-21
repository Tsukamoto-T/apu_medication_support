#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
#include <time.h>
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
#include <geometry_msgs/PointStamped.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Int16.h>

typedef pcl::PointXYZRGB PointT;

class IdentifyCalendar{
private:
  ros::Subscriber sub_points;
  ros::Subscriber sub_flag;
  ros::Publisher pub_all;
  ros::Publisher pub_rest;
  ros::Publisher pub_rest_removal;
  ros::Publisher pub_cluster_calendar;
  ros::Publisher pub_upper_left_point;
  ros::Publisher pub_present_point;
  ros::Publisher pub_pocket_points;

  void CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  void flagCb(const std_msgs::Int16 flag_i);

public:
  float first_plane_segmentation_threshold = 0.005;
  int outlier_removal_number_neighbors_wall = 20;
  double outlier_removal_th_wall = 0.1;
  float cluster_tolerance = 0.005;
  int max_cluster_size = 20000;
  int min_cluster_size = 500;
  int outlier_removal_number_neighbors_calendar = 20;
  double outlier_removal_th_calendar = 0.1;
  double x_base = 0.04;
  double y_base = 0.05;
  double x_interval = 0.0775;
  double y_interval = 0.06;
  int set_week = 1;
  int set_time = 7;

  std::string header_tf = "head_rgbd_sensor_rgb_frame";
  std_msgs::Header header;
  //std::string header_tf = "camera_depth_optical_frame";
  int flag = 0;

  IdentifyCalendar();
  void Output_pub(pcl::PointCloud<PointT>::Ptr cloud_ ,sensor_msgs::PointCloud2 msgs_,ros::Publisher pub_);
};

IdentifyCalendar::IdentifyCalendar(){
  ros::NodeHandle nh("~");
  nh.param<float>("first_plane_segmentation_threshold", first_plane_segmentation_threshold, first_plane_segmentation_threshold);
  nh.param<int>("outlier_removal_number_neighbors_wall",outlier_removal_number_neighbors_wall,outlier_removal_number_neighbors_wall);
  nh.param<double>("outlier_removal_th_wall",outlier_removal_th_wall,outlier_removal_th_wall);
  nh.param<float>("cluster_tolerance",cluster_tolerance,cluster_tolerance);
  nh.param<int>("max_cluster_size",max_cluster_size,max_cluster_size);
  nh.param<int>("min_cluster_size",min_cluster_size,min_cluster_size);
  nh.param<int>("outlier_removal_number_neighbors_calendar",outlier_removal_number_neighbors_calendar,outlier_removal_number_neighbors_calendar);
  nh.param<double>("outlier_removal_th_calendar",outlier_removal_th_calendar,outlier_removal_th_calendar);
  nh.param<double>("x_base",x_base,x_base);
  nh.param<double>("y_base",y_base,y_base);
  nh.param<double>("x_interval",x_interval,x_interval);
  nh.param<double>("y_interval",y_interval,y_interval);
  nh.param<int>("set_week",set_week,set_week);
  nh.param<int>("set_time",set_time,set_time);

  std::string hsr_topic = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";
  //std::string hsr_topic = "/camera/depth/color/points";
  sub_points = nh.subscribe(hsr_topic, 1, &IdentifyCalendar::CloudCb,this);
  sub_flag = nh.subscribe("/apu_identify_calendar_node/flag", 1, &IdentifyCalendar::flagCb,this);
  //if (!ros::topic::waitForMessage<sensor_msgs::PointCloud2>(hsr_topic, nh, ros::Duration(10.0))) {
  //if (!ros::topic::waitForMessage<sensor_msgs::PointCloud2>(hsr_topic)) {
  //  ROS_ERROR("timeout exceeded while waiting for message on topic %s", hsr_topic.c_str());
  //  exit(EXIT_FAILURE);
  //}

  pub_all = nh.advertise<sensor_msgs::PointCloud2>("cloud_all", 1);
  pub_rest = nh.advertise<sensor_msgs::PointCloud2>("cloud_rest", 1);
  pub_rest_removal = nh.advertise<sensor_msgs::PointCloud2>("cloud_rest_removal", 1);
  pub_cluster_calendar = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_calendar", 1);
  pub_upper_left_point = nh.advertise<sensor_msgs::PointCloud2>("cloud_upper_left_point", 1);
  pub_present_point = nh.advertise<sensor_msgs::PointCloud2>("cloud_present_point", 1);
  pub_pocket_points = nh.advertise<sensor_msgs::PointCloud2>("cloud_pocket_points", 1);

  header.frame_id = header_tf;
  header.stamp = ros::Time::now();
  x_base = x_base - 0.001;
  y_base = y_base - 0.002;
}

//====flag=============================================================================
void IdentifyCalendar::flagCb(const std_msgs::Int16 flag_i) {
  flag = flag_i.data;
}

//========出力関数=========================================================================
void IdentifyCalendar::Output_pub(pcl::PointCloud<PointT>::Ptr cloud_,sensor_msgs::PointCloud2 msgs_,ros::Publisher pub_){
  if(cloud_->size() <= 0){
    ROS_INFO("Size (%zu)",cloud_->size());
  }
  pcl::toROSMsg(*cloud_, msgs_);
  msgs_.header = header;
  pub_.publish(msgs_);
}

//=========コールバック==============================================================
void IdentifyCalendar::CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  std::cout << "identify calendar callback_start" << std::endl;
  if(!(flag == 1)){
    std::cout << "flag is false" << std::endl;
    return;
  }
  header = cloud_msg->header;
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
  seg.setMaxIterations (200);  //試行回数
  seg.setDistanceThreshold (first_plane_segmentation_threshold);  //距離の閾値

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_input_rm);
  seg.segment (*inliers, *coefficients);

  //=======壁平面除去============================================================
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud_input_rm); //入力データ
  extract.setIndices (inliers); //インデックスの入力
  //pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
  //extract.setNegative (false);  //内部の負のパラメータの値を返す（平面のみ出力）
  //extract.filter (*cloud_plane);  //入力データ内の平面データ
  pcl::PointCloud<PointT>::Ptr cloud_rest(new pcl::PointCloud<PointT>);
  extract.setNegative (true);  //trueの場合入力インデックスを除くすべてのポイントを返す
  extract.filter (*cloud_rest);

  //=====外れ値処理=====================================================================
  pcl::PointCloud<PointT>::Ptr cloud_rest_removal(new pcl::PointCloud<PointT>);
  pcl::StatisticalOutlierRemoval<PointT> sor_plane;
  sor_plane.setInputCloud(cloud_rest);
  sor_plane.setMeanK(outlier_removal_number_neighbors_wall);
  sor_plane.setStddevMulThresh(outlier_removal_th_wall);
  sor_plane.filter(*cloud_rest_removal);

  //======カレンダー以外の点群を除去==================================================
  //------クラスタリング-------------------------------------------------------------
  pcl::search::KdTree<PointT>::Ptr cluster_tree (new pcl::search::KdTree<PointT>);
  cluster_tree->setInputCloud(cloud_rest_removal);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (cluster_tolerance);//探索する半径の設定
  ec.setMinClusterSize (min_cluster_size);//最小点の数を設定
  ec.setMaxClusterSize (max_cluster_size);//最大の点の数を設定
  ec.setSearchMethod (cluster_tree);//検索先のポインタを指定
  ec.setInputCloud (cloud_rest_removal);//点群を入力
  ec.extract (cluster_indices);//クラスター情報を出力


  int j = 0;
  pcl::PointCloud<PointT>::Ptr cloud_cluster_calendar (new pcl::PointCloud<PointT>);

  for (std::vector<pcl::PointIndices>::const_iterator ite = cluster_indices.begin();ite != cluster_indices.end(); ++ite){
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

    for(std::vector<int>::const_iterator pite = ite->indices.begin(); pite != ite->indices.end(); pite++){
      cloud_cluster->points.push_back(cloud_rest_removal->points[*pite]);
    }

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    //---最も大きいクラスタをカレンダーとする---------------------------------------------------------
    if(0 < cloud_cluster->size() && j == 0){
      *cloud_cluster_calendar = *cloud_cluster;
    }
    j++;
  }

  if (cloud_cluster_calendar->size() <= 0){
    std::cout << "薬カレンダーを見つけられませんでした" << std::endl;
    return;
  }
  //=====外れ値処理=====================================================================
  //pcl::StatisticalOutlierRemoval<PointT> sor;
  //sor.setInputCloud(cloud_cluster_calendar);
  //sor.setMeanK(outlier_removal_number_neighbors_calendar);
  //sor.setStddevMulThresh(outlier_removal_th_calendar);
  //sor.filter(*cloud_cluster_calendar);

  //==========出力==================================================================
  sensor_msgs::PointCloud2 msgs_all;
  pcl::toROSMsg(*cloud_input_rm, msgs_all);
  pub_all.publish(msgs_all);

  sensor_msgs::PointCloud2 msgs_rest;
  pcl::toROSMsg(*cloud_rest, msgs_rest);
  pub_rest.publish(msgs_rest);

  sensor_msgs::PointCloud2 msgs_rest_removal;
  pcl::toROSMsg(*cloud_rest_removal, msgs_rest_removal);
  pub_rest_removal.publish(msgs_rest_removal);

  sensor_msgs::PointCloud2 msgs_cluster_calendar;
  //IdentifyCalendar::Output_pub(cloud_cluster_calendar,msgs_cluster_calendar,pub_cluster_calendar);
  pcl::toROSMsg(*cloud_cluster_calendar, msgs_cluster_calendar);
  msgs_cluster_calendar.header = header;
  pub_cluster_calendar.publish(msgs_cluster_calendar);


  //===========現在の時点にあったポケットの位置の推定=================================================================

  //----------カレンダーの左上の座標を求める----------------------------------------------
  double min_point = 0.0;
  double max_point = 0.0;
  int min_i, max_i;

  for(size_t i = 0; i < cloud_cluster_calendar->size(); ++i){
    if(min_point > (cloud_cluster_calendar->points[i].x + cloud_cluster_calendar->points[i].y)){
      min_point = cloud_cluster_calendar->points[i].x + cloud_cluster_calendar->points[i].y;
      min_i=(int)i;
    }
    if(max_point > (-(cloud_cluster_calendar->points[i].x) + cloud_cluster_calendar->points[i].y)){
      max_point = -(cloud_cluster_calendar->points[i].x) + cloud_cluster_calendar->points[i].y;
      max_i=(int)i;
    }
  }

  double calendar_x = abs(cloud_cluster_calendar->points[min_i].x - cloud_cluster_calendar->points[max_i].x);
  double calendar_y = abs(cloud_cluster_calendar->points[min_i].y - cloud_cluster_calendar->points[max_i].y);

  if(calendar_x < 0.38 || 0.4 < calendar_x || 0.05 < calendar_y){
    std::cout << "calendar point is not found" << std::endl;
    return;
  }

  //---出力----------------------------------------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upper_left_point(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ upper_left_point;
  upper_left_point.x = cloud_cluster_calendar->points[min_i].x;
  upper_left_point.y = cloud_cluster_calendar->points[min_i].y;
  upper_left_point.z = cloud_cluster_calendar->points[min_i].z;
  cloud_upper_left_point->points.push_back(upper_left_point);

  pcl::PointXYZ upper_right_point;
  upper_right_point.x = cloud_cluster_calendar->points[max_i].x;
  upper_right_point.y = cloud_cluster_calendar->points[max_i].y;
  upper_right_point.z = cloud_cluster_calendar->points[max_i].z;
  cloud_upper_left_point->points.push_back(upper_right_point);

  sensor_msgs::PointCloud2 msgs_upper_left_point;
  pcl::toROSMsg(*cloud_upper_left_point, msgs_upper_left_point);
  msgs_upper_left_point.header = header;
  pub_upper_left_point.publish(msgs_upper_left_point);

  //----現時点のポケットの位置を計算--------------------------------
  std::string week;
  std::string time;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_present_point(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ present_point;

  switch (set_week){
    case 1:
    present_point.y = upper_left_point.y + y_base;
    week = "月曜日";
    break;
    case 2:
    present_point.y = upper_left_point.y + y_base + y_interval;
    week = "火曜日";
    break;
    case 3:
    present_point.y = upper_left_point.y + y_base + y_interval * 2;
    week = "水曜日";
    break;
    case 4:
    present_point.y = upper_left_point.y + y_base + y_interval * 3;
    week = "木曜日";
    break;
    case 5:
    present_point.y = upper_left_point.y + y_base + y_interval * 4;
    week = "金曜日";
    break;
    case 6:
    present_point.y = upper_left_point.y + y_base + y_interval * 5;
    week = "土曜日";
    break;
    case 0:
    present_point.y = upper_left_point.y + y_base + y_interval * 6;
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
    present_point.x = upper_left_point.x + x_base;
    time = "あさ";
    break;

    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    present_point.x = upper_left_point.x + x_base + x_interval;
    time = "ひる";
    break;

    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    present_point.x = upper_left_point.x + x_base + x_interval * 2;
    time = "よる";
    break;

    case 22:
    case 23:
    case 0:
    case 1:
    case 2:
    case 3:
    present_point.x = upper_left_point.x + x_base + x_interval * 3;
    time = "ねる前";
    break;
  }

  present_point.z = upper_left_point.z - 0.01;
  cloud_present_point->points.push_back(present_point);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pocket_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ present_point_dr;
  present_point_dr.z = present_point.z;
  present_point.x = present_point.x - 0.01;
  present_point.y = present_point.y - 0.01;
  cloud_pocket_points->points.push_back(present_point);
  present_point_dr.x = present_point.x + x_interval + 0.02;
  present_point_dr.y = present_point.y + y_interval + 0.02;
  cloud_pocket_points->points.push_back(present_point_dr);

  //-----一時的に------------
  for(int i =0;i<3;i++){
    present_point.x = present_point.x + x_interval ;
    cloud_pocket_points->points.push_back(present_point);
    present_point_dr.x = present_point_dr.x + x_interval;
    cloud_pocket_points->points.push_back(present_point_dr);
  }


  //---出力-----
  sensor_msgs::PointCloud2 msgs_present_point;
  pcl::toROSMsg(*cloud_present_point, msgs_present_point);
  msgs_present_point.header = header;
  pub_present_point.publish(msgs_present_point);

  sensor_msgs::PointCloud2 msgs_pocket_points;
  pcl::toROSMsg(*cloud_pocket_points, msgs_pocket_points);
  msgs_pocket_points.header = header;
  pub_pocket_points.publish(msgs_pocket_points);

}

//=======main===================================================================
int main (int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "IdentifyCalendar");
  IdentifyCalendar IdentifyCalendar;

  ros::Rate rate(1);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  //ros::spin();

	return 0;
}
