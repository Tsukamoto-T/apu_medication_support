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

typedef pcl::PointXYZRGB PointT;

class CaseDisturbance{
private:
  ros::Subscriber sub;
  ros::Publisher pub_all;
  ros::Publisher pub_plane;
  ros::Publisher pub_rest;
  ros::Publisher pub_rest_removal;

  ros::Publisher pub_cluster_0;
  ros::Publisher pub_cluster_1;
  ros::Publisher pub_cluster_2;

  ros::Publisher pub_cluster_points;
  ros::Publisher pub_cluster_calendar;

  void CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

public:
  float threshold_calendar = 0.005;
  int calendar_number_neighbors = 20;
  float threshold_case = 0.005;
  int case_number_neighbors = 20;
  float cluster_tolerance = 0.005;
  int max_cluster_size = 20000;
  int min_cluster_size = 500;

  CaseDisturbance();
  void Output_pub(pcl::PointCloud<PointT>::Ptr cloud_ ,sensor_msgs::PointCloud2 msgs_,ros::Publisher pub_);
};

CaseDisturbance::CaseDisturbance(){
  ros::NodeHandle nh("~");
  nh.param<float>("threshold_calendar", threshold_calendar, threshold_calendar);
  nh.param<int>("calendar_number_neighbors",calendar_number_neighbors,calendar_number_neighbors);
  nh.param<float>("threshold_case", threshold_case, threshold_case);
  nh.param<int>("case_number_neighbors",case_number_neighbors,case_number_neighbors);
  nh.param<float>("cluster_tolerance",cluster_tolerance,cluster_tolerance);
  nh.param<int>("max_cluster_size",max_cluster_size,max_cluster_size);
  nh.param<int>("min_cluster_size",min_cluster_size,min_cluster_size);

  std::string hsr_topic = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";
  sub = nh.subscribe(hsr_topic, 1, &CaseDisturbance::CloudCb,this);
  if (!ros::topic::waitForMessage<sensor_msgs::PointCloud2>(hsr_topic, ros::Duration(10.0))) {
    ROS_ERROR("timeout exceeded while waiting for message on topic %s", hsr_topic.c_str());
    exit(EXIT_FAILURE);
  }

  pub_all = nh.advertise<sensor_msgs::PointCloud2>("cloud_all", 1);
  pub_plane = nh.advertise<sensor_msgs::PointCloud2>("cloud_plane", 1);
  pub_rest = nh.advertise<sensor_msgs::PointCloud2>("cloud_rest", 1);
  pub_rest_removal = nh.advertise<sensor_msgs::PointCloud2>("cloud_rest_removal", 1);
  pub_cluster_0 = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_0", 1);
  pub_cluster_1 = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_1", 1);
  pub_cluster_2 = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_2", 1);
  pub_cluster_points = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_points", 1);
  pub_cluster_calendar = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_calendar", 1);

}
//========出力関数=========================================================================
void CaseDisturbance::Output_pub(pcl::PointCloud<PointT>::Ptr cloud_,sensor_msgs::PointCloud2 msgs_,ros::Publisher pub_){
  if(cloud_->size() <= 0){
    ROS_INFO("Size (%d)",cloud_);
  }
  pcl::toROSMsg(*cloud_, msgs_);
  msgs_.header.frame_id = "head_rgbd_sensor_rgb_frame";
  pub_.publish(msgs_);
}
//=========コールバック==============================================================
void CaseDisturbance::CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
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

  //================クラスタリング====================================================
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
  double min_cluster_point = 100.0;
  pcl::PointCloud<PointT>::Ptr cloud_cluster_calendar (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cluster_0 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cluster_1 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cluster_2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cluster_3 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_points(new pcl::PointCloud<pcl::PointXYZ>);

  int cc[3] = {10000,10000,10000};

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end(); ++it){
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++){
      cloud_cluster->points.push_back(cloud_rest_removal->points[*pit]);
    }

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    switch(j){
      case 0:
      cc[0] = cloud_cluster->points.size();
      pcl::copyPointCloud(*cloud_cluster,*cloud_cluster_0);
      break;

      case 1:
      cc[1] = cloud_cluster->points.size();
      pcl::copyPointCloud(*cloud_cluster,*cloud_cluster_1);
      break;

      case 2:
      cc[2] = cloud_cluster->points.size();
      pcl::copyPointCloud(*cloud_cluster,*cloud_cluster_2);
      break;
    }
    //std::cout << cloud_cluster->points.size() << std::endl;
    //----クラスタごと中心を求める---------------------------------------------------------
    if(0 < cloud_cluster->size()){
      pcl::PointXYZ cent_;
      cent_.x = 0.0;
      cent_.y = 0.0;
      cent_.z = 0.0;
      for(size_t i = 0; i < cloud_cluster->size(); i++){
        cent_.x += cloud_cluster->points[i].x;
        cent_.y += cloud_cluster->points[i].y;
        cent_.z += cloud_cluster->points[i].z;
      }
      cent_.x /= static_cast<int>(cloud_cluster->size());
      cent_.y /= static_cast<int>(cloud_cluster->size());
      cent_.z /= static_cast<int>(cloud_cluster->size());
      cloud_cluster_points->points.push_back(cent_);
      //---（0,0)に最も近い点（クラスタ）をさがし、そのクラスタを薬カレンダーとみなす----------------
      double cluster_dist;
      cluster_dist = sqrt(pow(cent_.x ,2) + pow(cent_.y,2));
      if (min_cluster_point > cluster_dist){
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

  sensor_msgs::PointCloud2 msgs_cluster_points;
  pcl::toROSMsg(*cloud_cluster_points, msgs_cluster_points);
  msgs_cluster_points.header.frame_id = "head_rgbd_sensor_rgb_frame";
  pub_cluster_points.publish(msgs_cluster_points);

  sensor_msgs::PointCloud2 msgs_cluster_0;
  CaseDisturbance::Output_pub(cloud_cluster_0,msgs_cluster_0,pub_cluster_0);

  sensor_msgs::PointCloud2 msgs_cluster_1;
  CaseDisturbance::Output_pub(cloud_cluster_1,msgs_cluster_1,pub_cluster_1);

  sensor_msgs::PointCloud2 msgs_cluster_2;
  CaseDisturbance::Output_pub(cloud_cluster_2,msgs_cluster_2,pub_cluster_2);

  sensor_msgs::PointCloud2 msgs_cluster_calendar;
  CaseDisturbance::Output_pub(cloud_cluster_calendar,msgs_cluster_calendar,pub_cluster_calendar);
}

//=======main===================================================================
int main (int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "CaseDisturbance");
  CaseDisturbance CaseDisturbance;

  ros::Rate rate(1);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

	return 0;
}
