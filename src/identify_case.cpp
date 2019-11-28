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
  ros::Publisher pub_rest_trans;
  ros::Publisher pub_calendar;
  ros::Publisher pub_case;;
  ros::Publisher pub_case_removal;
  ros::Publisher pub_point;

  ros::Publisher pub_cluster_0;
  ros::Publisher pub_cluster_1;
  ros::Publisher pub_cluster_2;
  ros::Publisher pub_cluster_3;
  ros::Publisher pub_cluster_4;
  ros::Publisher pub_cluster_5;
  ros::Publisher pub_cluster_6;
  ros::Publisher pub_cluster_7;

  ros::Publisher pub_centroid_0;

  void CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

public:
  float threshold_calendar = 0.005;
  int calendar_number_neighbors = 20;
  float threshold_case = 0.005;
  int case_number_neighbors = 20;
  float cluster_tolerance = 0.005;
  int max_cluster_size = 20000;
  IdentifyCase();
  pcl::PointCloud<PointT>::Ptr Down_Sampling(pcl::PointCloud<PointT>::Ptr cloud_input);
  void Output_pub(pcl::PointCloud<PointT>::Ptr cloud_ ,sensor_msgs::PointCloud2 msgs_,ros::Publisher pub_);
};

IdentifyCase::IdentifyCase(){
  ros::NodeHandle nh("~");
  nh.param<float>("threshold_calendar", threshold_calendar, threshold_calendar);
  nh.param<int>("calendar_number_neighbors",calendar_number_neighbors,calendar_number_neighbors);
  nh.param<float>("threshold_case", threshold_case, threshold_case);
  nh.param<int>("case_number_neighbors",case_number_neighbors,case_number_neighbors);
  nh.param<float>("cluster_tolerance",cluster_tolerance,cluster_tolerance);
  nh.param<int>("max_cluster_size",max_cluster_size,max_cluster_size);

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
  pub_rest_trans = nh.advertise<sensor_msgs::PointCloud2>("cloud_rest_trans", 1);
  pub_calendar = nh.advertise<sensor_msgs::PointCloud2>("cloud_calendar", 1);
  pub_case = nh.advertise<sensor_msgs::PointCloud2>("cloud_case", 1);
  pub_case_removal = nh.advertise<sensor_msgs::PointCloud2>("cloud_case_removal", 1);
  pub_point = nh.advertise<sensor_msgs::PointCloud2>("cloud_point", 1);

  pub_cluster_0 = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_0", 1);
  pub_cluster_1 = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_1", 1);
  pub_cluster_2 = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_2", 1);
  pub_cluster_3 = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_3", 1);
  pub_cluster_4 = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_4", 1);
  pub_cluster_5 = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_5", 1);
  pub_cluster_6 = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_6", 1);
  pub_cluster_7 = nh.advertise<sensor_msgs::PointCloud2>("cloud_cluster_7", 1);

  pub_centroid_0 = nh.advertise<sensor_msgs::PointCloud2>("cloud_centroid_0", 1);

}

//======================ダウンサンプリング&特徴点抽出===============================================
pcl::PointCloud<PointT>::Ptr IdentifyCase::Down_Sampling(pcl::PointCloud<PointT>::Ptr  cloud_input)
{
  //==========ダウンサンプリング=======================================================
  pcl::PointCloud<PointT>::Ptr cloud_vg (new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud (cloud_input);
  vg.setLeafSize (0.005f, 0.005f, 0.005f); //リーフサイズ0.5cm
  vg.filter (*cloud_vg);

  return cloud_vg;
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

//=========コールバック==============================================================
void IdentifyCase::CloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  std::cout << "callback_start" << std::endl;
  pcl::PointCloud<PointT>::Ptr cloud_all(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_input_rm(new pcl::PointCloud<PointT>);
  std::vector<int> index_all;
  pcl::fromROSMsg(*cloud_msg, *cloud_all);
  pcl::removeNaNFromPointCloud(*cloud_all,*cloud_input_rm,index_all);

  pcl::PointCloud<PointT>::Ptr cloud_dwnsmp(new pcl::PointCloud<PointT>);
  cloud_dwnsmp = IdentifyCase::Down_Sampling(cloud_input_rm);

  //=======壁平面検出==============================================================
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  //pcl::PCDWriter writer; //?
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);  //試行回数
  seg.setDistanceThreshold (threshold_calendar);  //距離の閾値

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_dwnsmp);
  seg.segment (*inliers, *coefficients);
  //=======壁平面除去=============================================================
  pcl::PointCloud<PointT>::Ptr cloud_rest(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
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

  //=====外れ値処理=====================================================================
  pcl::PointCloud<PointT>::Ptr cloud_rest_removal(new pcl::PointCloud<PointT>);
  pcl::StatisticalOutlierRemoval<PointT> sor_plane;
  sor_plane.setInputCloud(cloud_rest);
  sor_plane.setMeanK(calendar_number_neighbors);
  sor_plane.setStddevMulThresh(1.0);
  sor_plane.filter(*cloud_rest_removal);

  //=======カレンダー平面を検出（ケースの検出）===========================================
  pcl::SACSegmentation<PointT> seg_2;
  pcl::PointIndices::Ptr inliers_2 (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_2 (new pcl::ModelCoefficients);
  seg_2.setOptimizeCoefficients (true);
  seg_2.setModelType (pcl::SACMODEL_PLANE);
  seg_2.setMethodType (pcl::SAC_RANSAC);
  seg_2.setMaxIterations (100);  //試行回数
  seg_2.setDistanceThreshold (threshold_case);  //距離の閾値
  seg_2.setInputCloud (cloud_rest_removal);
  seg_2.segment (*inliers_2, *coefficients_2);

  //カレンダー平面を求める=================================================================
  pcl::PointCloud<PointT>::Ptr cloud_calendar_pre(new pcl::PointCloud<PointT>);
  pcl::ExtractIndices<PointT> extract_2;
  extract_2.setInputCloud (cloud_rest_removal);
  extract_2.setIndices (inliers_2);
  extract_2.setNegative (false);
  extract_2.filter (*cloud_calendar_pre);

  //===カレンダーの重心を原点に移動===================================================
  //---計算----------------------------------------------------------------------
  tf::Transform transform_translation;
	static tf::TransformBroadcaster tf_bc_translation;

  double x_plane  = 0.0;
	double y_plane  = 0.0;
	double z_plane  = 0.0;
	for(size_t i = 0; i< cloud_calendar_pre->size(); ++i)
	{
		x_plane += cloud_calendar_pre->points[i].x;
		y_plane += cloud_calendar_pre->points[i].y;
		z_plane += cloud_calendar_pre->points[i].z;
	}

  x_plane /= static_cast<int>(cloud_calendar_pre->size());
  y_plane /= static_cast<int>(cloud_calendar_pre->size());
  z_plane /= static_cast<int>(cloud_calendar_pre->size());

  //---移動後のカレンダーにtfを貼る--------------------------------------------------------------
	transform_translation.setOrigin(tf::Vector3(x_plane, y_plane, z_plane));
	transform_translation.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	tf_bc_translation.sendTransform(tf::StampedTransform(transform_translation, ros::Time::now(), "head_rgbd_sensor_rgb_frame", "trans_frame"));

  //----座標移動-----------------------------------------------------------------
  Eigen::Affine3f affine_translation = Eigen::Affine3f::Identity();
	affine_translation.translation() << -x_plane, -y_plane, -z_plane;
  pcl::PointCloud<PointT>::Ptr cloud_rest_trans(new pcl::PointCloud<PointT>);
	pcl::transformPointCloud(*cloud_rest_removal, *cloud_rest_trans, affine_translation);

  //カレンダー平面除去=================================================================
  pcl::PointCloud<PointT>::Ptr cloud_calendar(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_case(new pcl::PointCloud<PointT>);
  extract_2.setInputCloud (cloud_rest_trans);
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

  //===カレンダーの左上の座標を求める=====================================================
  double min_sum_point = 0.0;
  int min_i;

  for(size_t i = 0; i< cloud_calendar->size(); ++i)
  {
    if(min_sum_point > (cloud_calendar->points[i].x + cloud_calendar->points[i].y)){
      min_sum_point = cloud_calendar->points[i].x + cloud_calendar->points[i].y;
      min_i=(int)i;
    }
  }

  //---出力----------------------------------------------------------------------
  pcl::PointCloud<PointT>::Ptr st_point(new pcl::PointCloud<PointT>);
  pcl::PointXYZRGB point;
  point.x = cloud_calendar->points[min_i].x;
  point.y = cloud_calendar->points[min_i].y;
  point.z = cloud_calendar->points[min_i].z;
  point.r = cloud_calendar->points[min_i].r;
  point.g = cloud_calendar->points[min_i].g;
  point.b = cloud_calendar->points[min_i].b;
  st_point->points.push_back(point);

  //================クラスタリング====================================================
  pcl::search::KdTree<PointT>::Ptr cluster_tree (new pcl::search::KdTree<PointT>);
  cluster_tree->setInputCloud(cloud_case_removal);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (cluster_tolerance);//探索する半径の設定
  ec.setMinClusterSize (70);//最小点の数を設定
  ec.setMaxClusterSize (max_cluster_size);//最大の点の数を設定
  ec.setSearchMethod (cluster_tree);//検索先のポインタを指定
  ec.setInputCloud (cloud_case_removal);//点群を入力
  ec.extract (cluster_indices);//クラスター情報を出力

  int j = 0;

  pcl::PointCloud<PointT>::Ptr cloud_cluster_0 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cluster_1 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cluster_2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cluster_3 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cluster_4 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cluster_5 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cluster_6 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cluster_7 (new pcl::PointCloud<PointT>);

  int cc[8] = {10000,10000,10000,10000,10000,10000,10000,10000};

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end(); ++it){
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++){
      cloud_cluster->points.push_back(cloud_case_removal->points[*pit]);
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

      case 3:
      cc[3] = cloud_cluster->points.size();
      pcl::copyPointCloud(*cloud_cluster,*cloud_cluster_3);
      break;

      case 4:
      cc[4] = cloud_cluster->points.size();
      pcl::copyPointCloud(*cloud_cluster,*cloud_cluster_4);
      break;

      case 5:
      cc[5] = cloud_cluster->points.size();
      pcl::copyPointCloud(*cloud_cluster,*cloud_cluster_5);
      break;

      case 6:
      cc[6] = cloud_cluster->points.size();
      pcl::copyPointCloud(*cloud_cluster,*cloud_cluster_6);
      break;

      case 7:
      cc[7] = cloud_cluster->points.size();
      pcl::copyPointCloud(*cloud_cluster,*cloud_cluster_7);
      break;

    }
    j++;
  }
  std::cout<<" "<<cloud_cluster_0->points.size()<<" "<<cloud_cluster_1->points.size()<<" "<<cloud_cluster_2->points.size()<<" "<<cloud_cluster_3->points.size()<<" "<<cloud_cluster_4->points.size()<<" "<<cloud_cluster_5->points.size()<<" "<<cloud_cluster_6->points.size()<<" "<<cloud_cluster_7->points.size()<<std::endl;

  //========クラスタごとの重心を求める==================================================
  Eigen::Vector4f centroid_0;
  pcl::compute3DCentroid(*cloud_cluster_0,centroid_0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centroid_0(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ cent_0;
  cent_0.x = centroid_0[0];
  cent_0.y = centroid_0[1];
  cent_0.z = centroid_0[2];
  cloud_centroid_0->points.push_back(cent_0);



  //==========出力==================================================================
  sensor_msgs::PointCloud2 msgs_all;
  pcl::toROSMsg(*cloud_dwnsmp, msgs_all);
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

  sensor_msgs::PointCloud2 msgs_rest_trans;
  pcl::toROSMsg(*cloud_rest_trans, msgs_rest_trans);
  pub_rest_trans.publish(msgs_rest_trans);

  sensor_msgs::PointCloud2 msgs_calendar;
  pcl::toROSMsg(*cloud_calendar, msgs_calendar);
  pub_calendar.publish(msgs_calendar);

  sensor_msgs::PointCloud2 msgs_case;
  pcl::toROSMsg(*cloud_case, msgs_case);
  pub_case.publish(msgs_case);

  sensor_msgs::PointCloud2 msgs_case_removal;
  pcl::toROSMsg(*cloud_case_removal, msgs_case_removal);
  pub_case_removal.publish(msgs_case_removal);

  sensor_msgs::PointCloud2 msgs_point;
  pcl::toROSMsg(*st_point, msgs_point);
  msgs_point.header.frame_id = "head_rgbd_sensor_rgb_frame";
  pub_point.publish(msgs_point);

  sensor_msgs::PointCloud2 msgs_centroid_0;
  pcl::toROSMsg(*cloud_centroid_0, msgs_centroid_0);
  msgs_centroid_0.header.frame_id = "head_rgbd_sensor_rgb_frame";
  pub_centroid_0.publish(msgs_centroid_0);

  //クラスタの出力-------------------------------------------------------------------
  sensor_msgs::PointCloud2 msgs_cluster_0;
  IdentifyCase::Output_pub(cloud_cluster_0,msgs_cluster_0,pub_cluster_0);
  sensor_msgs::PointCloud2 msgs_cluster_1;
  IdentifyCase::Output_pub(cloud_cluster_1,msgs_cluster_1,pub_cluster_1);
  sensor_msgs::PointCloud2 msgs_cluster_2;
  IdentifyCase::Output_pub(cloud_cluster_2,msgs_cluster_2,pub_cluster_2);
  sensor_msgs::PointCloud2 msgs_cluster_3;
  IdentifyCase::Output_pub(cloud_cluster_3,msgs_cluster_3,pub_cluster_3);
  sensor_msgs::PointCloud2 msgs_cluster_4;
  IdentifyCase::Output_pub(cloud_cluster_4,msgs_cluster_4,pub_cluster_4);
  sensor_msgs::PointCloud2 msgs_cluster_5;
  IdentifyCase::Output_pub(cloud_cluster_5,msgs_cluster_5,pub_cluster_5);
  sensor_msgs::PointCloud2 msgs_cluster_6;
  IdentifyCase::Output_pub(cloud_cluster_6,msgs_cluster_6,pub_cluster_6);
  sensor_msgs::PointCloud2 msgs_cluster_7;
  IdentifyCase::Output_pub(cloud_cluster_7,msgs_cluster_7,pub_cluster_7);
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
