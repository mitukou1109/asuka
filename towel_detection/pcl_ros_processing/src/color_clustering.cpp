#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
//以下追記
#include <pcl/filters/passthrough.h>  //PassThroughフィルタ
#include <pcl/filters/voxel_grid.h> //VoxelGridフィルタ
#include <pcl/common/common.h>  //クラスタリング用
#include <pcl/kdtree/kdtree.h>  //クラスタリング用
#include <pcl/segmentation/extract_clusters.h>  //クラスタリング用
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>
#include "std_msgs/Bool.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
double limit_z_min = 0.005;
int max_h, min_h, max_s, min_s, max_v, min_v;
bool nearest_flag = false;
double width_max=0.8 ,width_min=0.0, ignore_size=500;
double scale_x,scale_y,pose_x,pose_y;
double cut_scale = 0.18,recluster_scale=0.4;
bool omelette=false;
bool color_output = false;


class ClusteringNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_towels, sub_mode;
  std::string target_frame_;
  tf::TransformListener tf_listener_;
  ros::Publisher pub_transformed_,pub_myT,pub_others;
  PointCloud::Ptr cloud_tranformed_,cloud_filtered,myT_cloud,others_cloud;
  bool remove_flag;


  /*以下を追記　PassThroughフィルタ
  PassThroughフィルタ：得られた点群のうち、一定の範囲内にある点群のみを抽出する*/
  pcl::PassThrough<PointT> pass_z,pass_x,pass_nearest_x,pass_nearest_y; //PassThroughフィルタの宣言
  ros::Publisher pub_passthrough_,pub_passthrough_second,pub_passthrough_third;

  // 以下を追記　VoxcelGridフィルタ
  //VoxelGird:点群のダウンサンプリング
  pcl::VoxelGrid<PointT> voxel_;//Voxelフィルタの宣言

  // 以下を追記　クラスタリング用
  pcl::EuclideanClusterExtraction<PointT> ec_myT,ec_NmyT,ec_others;
  pcl::search::KdTree<PointT>::Ptr tree_;
  ros::Publisher pub_towel_clusters_;


  void judge_clusters(double base,int indent,visualization_msgs::Marker marker, visualization_msgs::MarkerArray marker_array, int* target_index)
  {
    if(*target_index < indent)
    {
      *target_index = marker_array.markers.size();
    }
    else
    {
      float d1 = fabsf(base - marker_array.markers[*target_index].pose.position.y);
      float d2 = fabsf(base - marker.pose.position.y);

      if(d2<d1)
      {
        *target_index = marker_array.markers.size();
      }
    }
  }

  void cb_mode(const std_msgs::Bool &mode)
  {
    omelette= mode.data;
  }

  //----------------------------------------------------------------
  //バスタオルへの処理
  void cb_towels(const PointCloud::ConstPtr &msg)
  {
    ROS_INFO("Let's color clustering");
    try
    {
      std::string frame_id = msg->header.frame_id;
      PointCloud::ConstPtr cloud_src = msg;
      if (target_frame_.empty() == false) //座標変換の指定がある場合、点群の座標変換を行う
      {
        frame_id = target_frame_;
        if (pcl_ros::transformPointCloud(
                target_frame_, *msg, *cloud_tranformed_, tf_listener_) == false)
        {
          ROS_ERROR("Failed pcl_ros::transformPointCloud. target_frame = %s",
                    target_frame_.c_str());
          return;
        }
        pub_transformed_.publish(cloud_tranformed_);
        cloud_src = cloud_tranformed_;
      }

      // 以下 cloud_src に対するフィルタ処理を書く
      //PassThroughフィルタ
      pcl::IndicesPtr indices (new std::vector <int>);

      pass_z.setFilterLimits(limit_z_min,1.5);
      pass_z.setInputCloud(cloud_src);
      pass_z.filter(*cloud_filtered);
      pass_x.setInputCloud(cloud_filtered);
      pass_x.filter(*cloud_filtered);

      pub_passthrough_.publish(cloud_filtered);

      //以下 VoxelGridフィルタ
      voxel_.setInputCloud(cloud_filtered);
      voxel_.filter(*cloud_filtered);

      //以下　カラーフィルター
      pcl::PointCloud<pcl::PointXYZHSV> hsv_cloud;
      PointCloudXYZRGBAtoXYZHSV(*cloud_filtered, hsv_cloud);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      int num=0,sum_h=0;
      // color filter（該当する色の点群をまとめる）
      for (size_t i = 0; i < cloud_filtered->points.size(); ++i){
        if (hsv_cloud.points[i].h > min_h && hsv_cloud.points[i].h < max_h &&
            hsv_cloud.points[i].s > min_s && hsv_cloud.points[i].s < max_s &&
            hsv_cloud.points[i].v > min_v && hsv_cloud.points[i].v < max_v
           ){

          if (color_output)
          {
            sum_h+=hsv_cloud.points[i].h;
            num++;
          }

          inliers->indices.emplace_back(i);
        }
      }
      if (color_output&&sum_h!=0&&num!=0)
      {
        ROS_INFO("h:%d",sum_h/num);
      }

      //色分け
      pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
      pcl::copyPointCloud(*cloud_filtered, *myT_cloud);
      pcl::copyPointCloud(*cloud_filtered, *others_cloud);

      extract.setInputCloud(myT_cloud);
      extract.setIndices(inliers);
      extract.setNegative(false); //自チームの色のものだけを残す
      extract.filter(*myT_cloud);  //→ｍｙT_cloudに保存
      pub_myT.publish(myT_cloud);

      extract.setInputCloud(others_cloud);
      extract.setIndices(inliers);
      extract.setNegative(true);  //自チーム以外の物だけを残す
      extract.filter(*others_cloud);
      pub_others.publish(others_cloud);


      //以下クラスタリング(自チーム)
      std::vector<pcl::PointIndices> cluster_indices_myT,cluster_indices_others;
      tree_->setInputCloud(myT_cloud);
      ec_myT.setInputCloud(myT_cloud);
      ec_myT.extract(cluster_indices_myT);

      //以下、簡単化のためクラスタにマーカーを割り当てる
      visualization_msgs::MarkerArray marker_array,towel_array;
      int target_index = -1;  //振り分け用(最も近いクラスタの検出時)
      int marker_id = 0;
      size_t ok = 0;
      for(std::vector<pcl::PointIndices>::const_iterator
                it = cluster_indices_myT.begin(),
                it_end = cluster_indices_myT.end();
              it != it_end; ++it, ++marker_id)
      {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*myT_cloud, *it, min_pt, max_pt);
        Eigen::Vector4f cluster_size = max_pt - min_pt;
        if(cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z()>0)
        {
          //以下　特定の条件に合うクラスタの検出
          visualization_msgs::Marker marker = makeMarker(frame_id, "my_towel", marker_id, min_pt, max_pt, 0.0f, 1.0f, 0.0f, 0.5f);
          ROS_WARN("it size :%d",(int)it->indices.size());
          //オムライスモードへの対応（omelette==falseのとき、小さなクラスタは無視する. ただし手前にあるタオルは取りこぼしの可能性が高いので取得）
          if ((it->indices.size() < ignore_size) && (marker.pose.position.x > 0.2)) {
            marker.ns = "omelette";
            if (it->indices.size() < ignore_size*0.6)
            {
              marker.ns = "danger_omelette";
            }

            if (omelette == false){continue;}
          }

          int cut_num = (int)(marker.scale.y/cut_scale );//切り捨て
          if (cut_num<=0) { cut_num = 1; }
          double scale_y = marker.scale.y;
          double cut_width = scale_y / cut_num;
          for (size_t i = 0; i < cut_num; i++) {
            Eigen::Vector4f max = max_pt, min = min_pt;
            min.y() = min_pt.y() + cut_width * i;
            max.y() = min_pt.y() + cut_width * (i+1);
            std::string marker_ns = marker.ns;
            marker = makeMarker(frame_id, marker_ns, marker_id++, min, max, 0.0f, 1.0f, 0.0f, 0.5f);
            judge_clusters(0,0,marker, marker_array, &target_index);
            marker_array.markers.emplace_back(marker);
          }
        }
      }

      if(marker_array.markers.empty() == false)
      {
        //最も近いクラスタへの処理
        if (target_index >= 0)
        {
          scale_x = marker_array.markers[target_index].scale.x;
          pose_x = marker_array.markers[target_index].pose.position.x;
          scale_y = marker_array.markers[target_index].scale.y;
          pose_y = marker_array.markers[target_index].pose.position.y;

          std::string marker_ns = marker_array.markers[target_index].ns;
          if ((marker_ns == "omelette") || (marker_ns == "danger_omelette"))
          {
            remove_flag = true;
          }
          else{remove_flag = false;}

          visualization_msgs::Marker nearest_marker = marker_array.markers[target_index];

          if (scale_x > recluster_scale)
          {  //naname
            pass_nearest_x.setFilterLimits(pose_x - scale_x/2, pose_x + scale_x/2);
            pass_nearest_y.setFilterLimits(pose_y-scale_y/2, pose_y + scale_y/2);
            pass_nearest_x.setInputCloud(myT_cloud);
            pass_nearest_x.filter(*myT_cloud);
            pass_nearest_y.setInputCloud(myT_cloud);
            pass_nearest_y.filter(*myT_cloud);
            pub_passthrough_second.publish(myT_cloud);

            std::vector<pcl::PointIndices> Ncluster_indices;
            tree_->setInputCloud(myT_cloud);
            ec_NmyT.setInputCloud(myT_cloud);
            ec_NmyT.extract(Ncluster_indices);
            marker_id=0;
            for(std::vector<pcl::PointIndices>::const_iterator
                      it_n = Ncluster_indices.begin(),
                      it_n_end = Ncluster_indices.end();
                    it_n != it_n_end; ++it_n, ++marker_id)
            {
              Eigen::Vector4f min_pt_n, max_pt_n;
              pcl::getMinMax3D(*myT_cloud, *it_n, min_pt_n, max_pt_n);
              Eigen::Vector4f Ncluster_size = max_pt_n - min_pt_n;
              if(Ncluster_size.x() > 0 && Ncluster_size.y() > 0 && Ncluster_size.z()>0)
              {
                nearest_marker = makeMarker(frame_id, marker_ns, marker_id, min_pt_n, max_pt_n, 1.0f, 0.0f, 1.0f, 0.5f);
                scale_x = nearest_marker.scale.x;
                pose_x = nearest_marker.pose.position.x;
                // pub_myT_Ncluster.publish(nearest_marker);
                ROS_INFO("make nearest marker");
                // break;
              }
            }
            if (nearest_marker.id >0) {ROS_ERROR("marker id !=0 num:%d",marker_id);}
          }
          towel_array.markers.emplace_back(nearest_marker);
          nearest_flag = true;
        }
      }

      //自チームタオル以外へのクラスタリング
      if (nearest_flag) {
        //範囲の縮小
        if (remove_flag)
        {
          pass_nearest_x.setFilterLimits(width_min, pose_x + scale_x / 2 + 0.22);
          //pass_nearest_y.setFilterLimits(pose_y - scale_y / 2 - 0.15, pose_y + scale_y / 2 + 0.15);
        }
        else{
          pass_nearest_x.setFilterLimits(width_min, pose_x + scale_x / 2);
          //pass_nearest_y.setFilterLimits(pose_y - scale_y / 2 - 0.05, pose_y + scale_y / 2 + 0.05);
        }

        //if (scale_y < cut_scale*1.5) {  //removeを行う必要がある可能性が高い場合
          pass_nearest_y.setFilterLimits(pose_y - scale_y/2-0.15, pose_y + scale_y/2+0.15);
        //}
        //else{
          //pass_nearest_y.setFilterLimits(pose_y- scale_y/2, pose_y + scale_y/2);
        //}
        pass_nearest_x.setInputCloud(others_cloud);
        pass_nearest_x.filter(*others_cloud);
        pass_nearest_y.setInputCloud(others_cloud);
        pass_nearest_y.filter(*others_cloud);
        pub_passthrough_third.publish(others_cloud);

        tree_->setInputCloud(others_cloud);
        ec_others.setInputCloud(others_cloud);
        ec_others.extract(cluster_indices_others);
        target_index = -1;
        marker_id = 1;
        for(std::vector<pcl::PointIndices>::const_iterator
          itO= cluster_indices_others.begin(),
          itO_end = cluster_indices_others.end();
          itO != itO_end; ++itO, ++marker_id)
          {
            Eigen::Vector4f min_ptO, max_ptO;
            pcl::getMinMax3D(*others_cloud, *itO, min_ptO, max_ptO);
            Eigen::Vector4f cluster_sizeO = max_ptO - min_ptO;
            if(cluster_sizeO.x() > 0 && cluster_sizeO.y() > 0 && cluster_sizeO.z()>0)
            {
              //以下　特定の条件に合うクラスタの検出
              visualization_msgs::Marker marker = makeMarker(frame_id, "others", marker_id, min_ptO, max_ptO, 1.0f, 0.0f, 0.0f, 0.5f);
              judge_clusters(pose_y,1,marker, towel_array, &target_index);
              towel_array.markers.emplace_back(marker);
            }
          }
          if((int)towel_array.markers.size() > 1 )
          {
            if (target_index >= 1) {
              towel_array.markers[target_index].ns = "nearest_others";
              towel_array.markers[target_index].color.r = 1.0f;
              towel_array.markers[target_index].color.g = 0.3f;
              towel_array.markers[target_index].color.b = 0.3f;
              towel_array.markers[target_index].color.a = 0.7f;
            }
          }
          nearest_flag = false;
      }

      if(towel_array.markers.empty() == false)
      {
        pub_towel_clusters_.publish(towel_array);
      }
    }
    catch (std::exception &e)
    {
      ROS_ERROR("%s", e.what());
    }
  }

  visualization_msgs::Marker makeMarker(
      const std::string &frame_id, const std::string &marker_ns,
      int marker_id,
      const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
      float r, float g, float b, float a) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = marker_ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
    marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
    marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = max_pt.x() - min_pt.x();
    marker.scale.y = max_pt.y() - min_pt.y();
    marker.scale.z = max_pt.z() - min_pt.z();

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.lifetime = ros::Duration(0.3);
    return marker;
  }


public:
  ClusteringNode()
    : nh_()
    , pnh_("~")
  {
    std::string topic_name;
    pnh_.param("target_frame", target_frame_, std::string("base_link"));
    pnh_.param("topic_name", topic_name, std::string("/my_towels"));
    ROS_INFO("target_frame = '%s'", target_frame_.c_str());
    ROS_INFO("topic_name = '%s'", topic_name.c_str());
    sub_towels = nh_.subscribe(topic_name, 5, &ClusteringNode::cb_towels, this);
    sub_mode = nh_.subscribe("/omelette", 5, &ClusteringNode::cb_mode, this);
    pub_transformed_ = nh_.advertise<PointCloud>("cloud_transformed", 1);
    pub_myT = nh_.advertise<PointCloud>("my_towel", 1);
    pub_others = nh_.advertise<PointCloud>("others", 1);
    pub_passthrough_ = nh_.advertise<PointCloud>("passthrough_fast", 1);
    pub_passthrough_second = nh_.advertise<PointCloud>("passthrough_second", 1);
    pub_passthrough_third = nh_.advertise<PointCloud>("passthrough_third", 1);
    pub_towel_clusters_ = nh_.advertise<visualization_msgs::MarkerArray>("towel_clusters", 1);
    cloud_tranformed_.reset(new PointCloud());
    myT_cloud.reset(new PointCloud());
    others_cloud.reset(new PointCloud());

    //以下を追記　クラスタリング
    int size_min_myT = 200,size_min_others = 100;
    pnh_.getParam("size_min_myT", size_min_myT);
    pnh_.getParam("size_min_others", size_min_others);
    pnh_.getParam("width_max", width_max);
    pnh_.getParam("width_min", width_min);
    pnh_.getParam("limit_z_min", limit_z_min);
    double down_rate = 0.01;
    pnh_.getParam("down_rate", down_rate);
    pnh_.getParam("cut_scale", cut_scale);
    pnh_.getParam("recluster_scale", recluster_scale);
    pnh_.getParam("ignore_size", ignore_size);
    pnh_.getParam("color_output", color_output);

    ros::param::get("max_h", max_h);
    ros::param::get("min_h", min_h);
    ros::param::get("max_s", max_s);
    ros::param::get("min_s", min_s);
    ros::param::get("max_v", max_v);
    ros::param::get("min_v", min_v);

    // PassThroughフィルタ
    pass_z.setFilterFieldName("z");  // Z軸（前方）の値でフィルタをかける
    // pass_z.setFilterLimits(0.01,1.5);  // 0.3 ～ 0.85 m の間にある点群を抽出
    pass_x.setFilterFieldName("x");  // X軸（前方）の値でフィルタをかける
    pass_x.setFilterLimits(width_min,width_max);  // 0.3 ～ 0.85 m の間にある点群を抽出
    pass_nearest_x.setFilterFieldName("x");  // X軸（前方）の値でフィルタをかける
    pass_nearest_y.setFilterFieldName("y");  // X軸（前方）の値でフィルタをかける

    cloud_filtered.reset(new PointCloud());

    // VoxcelGridフィルタ
    voxel_.setLeafSize(down_rate, down_rate, down_rate); //0.015m間隔でダウンサンプリング


    ec_myT.setClusterTolerance(0.01);//15cm以上離れていれば別のクラスタだとみなす
    ec_myT.setMinClusterSize(size_min_myT);     //クラスタを構成する点の数(min)
    ec_myT.setMaxClusterSize(300000);  //クラスタを構成する点の数(max)
    ec_myT.setSearchMethod(tree_);   //ある点とクラスタを形成可能な点の探索方法としてKD木を使用する。
    ec_NmyT.setClusterTolerance(0.01);//15cm以上離れていれば別のクラスタだとみなす
    ec_NmyT.setMinClusterSize(size_min_myT);     //クラスタを構成する点の数(min)
    ec_NmyT.setMaxClusterSize(300000);  //クラスタを構成する点の数(max)
    ec_NmyT.setSearchMethod(tree_);   //ある点とクラスタを形成可能な点の探索方法としてKD木を使用する。
    ec_others.setClusterTolerance(0.01);//15cm以上離れていれば別のクラスタだとみなす
    ec_others.setMinClusterSize(size_min_others);     //クラスタを構成する点の数(min)
    ec_others.setMaxClusterSize(300000);  //クラスタを構成する点の数(max)
    ec_others.setSearchMethod(tree_);   //ある点とクラスタを形成可能な点の探索方法としてKD木を使用する。
    tree_.reset(new pcl::search::KdTree<PointT>());

  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "clustering_node");

  ClusteringNode pointcloud_test;
  ROS_INFO("Hello Point Cloud!");
  ros::spin();
}
