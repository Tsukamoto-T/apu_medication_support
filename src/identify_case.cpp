#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
#include <time.h>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/registration/icp.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <geometry_msgs/PointStamped.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/features/normal_3d_omp.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZRGB PointT;

class IdentifyCase{
private:
  ros::Subscriber sub;
  ros::Publisher pub_all;
  ros::Publisher pub_plane;
  ros::Publisher pub_rest;
  ros::Publisher pub_rest_removal;
  void CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

public:
  float threshold_plane = 0.005;
  int number_neighbors = 20;
  float seddev_multhresh = 1;
  IdentifyCase();
  pcl::PointCloud<PointT>::Ptr Down_Sampling(pcl::PointCloud<PointT>::Ptr cloud_input);
};

IdentifyCase::IdentifyCase(){
  ros::NodeHandle nh("~");
  nh.param<float>("threshold_plane", threshold_plane, threshold_plane);
  nh.param<int>("number_neighbors",number_neighbors,number_neighbors);
  nh.param<float>("seddev_multhresh",seddev_multhresh,seddev_multhresh);

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
}

//======================ダウンサンプリング===============================================
pcl::PointCloud<PointT>::Ptr IdentifyCase::Down_Sampling(pcl::PointCloud<PointT>::Ptr  cloud_input)
{
  pcl::PointCloud<PointT>::Ptr cloud_vg (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<PointT>::Ptr keypoints3D(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_output(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT> cloud_merged;

  //filterling 0.5cmのリーフサイズを使用してダウンサンプリング
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud (cloud_input);
  vg.setLeafSize (0.005f, 0.005f, 0.005f);
  vg.filter (*cloud_vg);

  //特徴点抽出
  pcl::HarrisKeypoint3D<PointT,pcl::PointXYZI> Harris;
  Harris.setInputCloud(cloud_input);
  Harris.setNonMaxSupression(true);
  Harris.setRadius(0.01);

  Harris.compute(*keypoints);

  PointT key; //新しく点を格納するところ

  double min=0.0;
  double max=0.0;

  //Harris検出器をかけていく
  for(pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i!=keypoints->end(); i++){
    key = PointT((*i).x,(*i).y,(*i).z);
    if ((*i).intensity>max ){
      max = (*i).intensity;
    }
    if ((*i).intensity<min){
      min = (*i).intensity;
    }
    keypoints3D->push_back(key);
  }

  cloud_merged = *cloud_vg;
  cloud_merged += *keypoints3D;

  pcl::copyPointCloud(cloud_merged,*cloud_output);
  return cloud_output;
}

void IdentifyCase::CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  std::cout << "callback_start" << std::endl;
  pcl::PointCloud<PointT>::Ptr cloud_all(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_input_rm(new pcl::PointCloud<PointT>);
  std::vector<int> index_all;
  pcl::fromROSMsg(*cloud_msg, *cloud_all);
  pcl::removeNaNFromPointCloud(*cloud_all,*cloud_input_rm,index_all);

  pcl::PointCloud<PointT>::Ptr cloud_dwnsmp(new pcl::PointCloud<PointT>);
  cloud_dwnsmp = IdentifyCase::Down_Sampling(cloud_input_rm);

  //=======平面除去(平面モデル,平面検出=======================================================
  // Create the segmentation object for the planar model and set all the parameters
  pcl::PointCloud<PointT>::Ptr cloud_rest(new pcl::PointCloud<PointT>);
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>());
  pcl::PCDWriter writer; //?
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);  //試行回数
  seg.setDistanceThreshold (threshold_plane);  //距離の閾値
  std::cout << threshold_plane << '\n';

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_dwnsmp);
  seg.segment (*inliers, *coefficients);
  //=======平面除去（入力でデータから平面を除去する）=======================================================
  // Extract the planar inliers from the input cloud（入力データから平面インライナを抽出）
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud_dwnsmp); //入力データ
  extract.setIndices (inliers); //インデックスの入力
  extract.setNegative (false);  //内部の負のパラメータの値を返す（平面のみ出力）
  // Get the points associated with the planar surface（平面に関連付けられたポイントの取得）
  extract.filter (*cloud_plane);  //入力データ内の平面データ

  // Remove the planar inliers, extract the rest（平面インライナーを除去し、残りの抽出）
  extract.setNegative (true);  //trueの場合入力インデックスを除くすべてのポイントを返す
  extract.filter (*cloud_rest);
  //std::cout << "PointCloud representing the rest component: " << cloud_rest->points.size () << " data points." << std::endl;

  //外れ値処理=====================================================================
  pcl::PointCloud<PointT>::Ptr cloud_rest_removal(new pcl::PointCloud<PointT>);
  pcl::StatisticalOutlierRemoval<PointT> sor_plane;
  sor_plane.setInputCloud(cloud_rest);
  sor_plane.setMeanK(number_neighbors);
  sor_plane.setStddevMulThresh(seddev_multhresh);
  sor_plane.filter(*cloud_rest_removal);


  std::cout <<"threshold_plane :"<< threshold_plane << '\n';
  //==========出力==================================================================
  sensor_msgs::PointCloud2 msgs_all;
  pcl::toROSMsg(*cloud_dwnsmp, msgs_all);
  //pub_cloud.header.frame_id = "head_rgbd_sensor_link";  //tf
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

}

//=======main===================================================================
int main (int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "IdentifyCase");
  IdentifyCase IdentifyCase;

  ros::spin();

	return 0;
}
