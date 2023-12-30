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

#define Catch_width 0.15

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
bool now_target_1 = false,now_target_2 = false;

class ClusteringNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_myT;
  ros::Subscriber sub_others,sub_cluster;
  std::string target_frame_;
  tf::TransformListener tf_listener_;
  ros::Publisher pub_transformed_;
  PointCloud::Ptr cloud_tranformed_myT,cloud_tranformed_others;

  /*以下を追記　PassThroughフィルタ
  PassThroughフィルタ：得られた点群のうち、一定の範囲内にある点群のみを抽出する*/
  pcl::PassThrough<PointT> pass_x,pass_y; //PassThroughフィルタの宣言
  PointCloud::Ptr cloud_passthrough_myT,cloud_passthrough_others;
  ros::Publisher pub_passthrough_myT,pub_passthrough_others;

  // 以下を追記　VoxcelGridフィルタ
  //VoxelGird:点群のダウンサンプリング
  // pcl::VoxelGrid<PointT> voxel_;//Voxelフィルタの宣言
  // PointCloud::Ptr cloud_voxel_;
  // ros::Publisher pub_voxel_;

  // 以下を追記　クラスタリング用cloud_passthrough_
  pcl::search::KdTree<PointT>::Ptr tree_;
  pcl::EuclideanClusterExtraction<PointT> ec_myT,ec_others;
  ros::Publisher pub_myT_clusters_,pub_others_clusters_;

  visualization_msgs::Marker target_towel_ptr;

  visualization_msgs::Marker judge_clusters(visualization_msgs::Marker marker, visualization_msgs::MarkerArray marker_array, int* target_index)
  {
    if(*target_index < 0)
    {
      *target_index = marker_array.markers.size();
    }
    else
    {
      float d1 = fabsf(marker_array.markers[*target_index].pose.position.y);
      float d2 = fabsf(marker.pose.position.y);

      if(d2<d1)
      {
        *target_index = marker_array.markers.size();
      }
    }

    return marker;
  }

  void cb_cluster(const visualization_msgs::MarkerArray &msg)
  {

    // const visualization_msgs::Marker *target = NULL;
    visualization_msgs::MarkerArray::_markers_type::const_iterator it,it_end;
    for (it = msg.markers.begin(),it_end = msg.markers.end();
         it != it_end; ++it)
    {
        const visualization_msgs::Marker &marker = *it;
        if (marker.ns == "nearest_towel")
        {
          target_towel_ptr = marker;
          // target_towel_ptr = *target;
          now_target_1 = true;
          now_target_2 = true;
        }
    }
    // ROS_INFO("myT_clusters: %zu", msg.markers.size());
  }

  //----------------------------------------------------------------
  //自チームバスタオルへの処理
  void cb_myT(const PointCloud::ConstPtr &msg)
  {
    if (now_target_1==true) {
      visualization_msgs::Marker target_towel = target_towel_ptr;

      try
      {
        std::string frame_id = msg->header.frame_id;
        PointCloud::ConstPtr cloud_src = msg;
        if (target_frame_.empty() == false) //データが入っているとき
        {
          frame_id = target_frame_;
          if (pcl_ros::transformPointCloud(
            target_frame_, *msg, *cloud_tranformed_myT, tf_listener_) == false)
            {
              ROS_ERROR("Failed pcl_ros::transformPointCloud. target_frame = %s",
              target_frame_.c_str());
              return;
            }
            pub_transformed_.publish(cloud_tranformed_myT);
            cloud_src = cloud_tranformed_myT;
          }
          // ここに cloud_src に対するフィルタ処理を書く
          //以下 PassThroughフィルタ
          pass_x.setFilterLimits(target_towel.pose.position.x - Catch_width/2, target_towel.pose.position.x + Catch_width/2);  // 0.3 ～ 0.85 m の間にある点群を抽出
          pass_y.setFilterLimits(target_towel.pose.position.y - target_towel.scale.y/2, target_towel.pose.position.y + target_towel.scale.y/2);  // 0.3 ～ 0.85 m の間にある点群を抽出
                  // pass_x.setFilterLimits(0,1.5);
                  // pass_y.setFilterLimits(0,1.5);

          pass_x.setInputCloud(cloud_src);
          pass_x.filter(*cloud_passthrough_myT);
          pass_y.setInputCloud(cloud_passthrough_myT);
          pass_y.filter(*cloud_passthrough_myT);
          pub_passthrough_myT.publish(cloud_passthrough_myT);

          //以下 VoxelGridフィルタ
          // voxel_.setInputCloud(cloud_passthrough_);
          // voxel_.filter(*cloud_voxel_);
          // pub_voxel_.publish(cloud_voxel_);

          //以下クラスタリング
          std::vector<pcl::PointIndices> cluster_indices;
          tree_->setInputCloud(cloud_passthrough_myT);
          ec_myT.setInputCloud(cloud_passthrough_myT);
          ec_myT.extract(cluster_indices);
          visualization_msgs::MarkerArray marker_array;
          int target_index = -1;  //振り分け用(最も近いクラスタの検出時)
          int marker_id = 0;
          size_t ok = 0;
          for(std::vector<pcl::PointIndices>::const_iterator
            it = cluster_indices.begin(),
            it_end = cluster_indices.end();
            it != it_end; ++it, ++marker_id)
            {
              Eigen::Vector4f min_pt, max_pt;
              pcl::getMinMax3D(*cloud_passthrough_myT, *it, min_pt, max_pt);
              Eigen::Vector4f cluster_size = max_pt - min_pt;
              if(cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z()>0)
              {
                //以下　特定の条件に合うクラスタの検出
                visualization_msgs::Marker marker = makeMarker(frame_id, "my_towel", marker_id, min_pt, max_pt, 0.0f, 1.0f, 0.0f, 0.5f);
                bool cut_flag = false;
                double cut_scale;

                marker = judge_clusters(marker, marker_array, &target_index);
                marker_array.markers.push_back(marker);
              }
            }
            if(marker_array.markers.empty() == false)
            {
              if (target_index >= 0) {
                marker_array.markers[target_index].ns = "nearest_towel";
                marker_array.markers[target_index].color.r = 1.0f;
                marker_array.markers[target_index].color.g = 0.0f;
                marker_array.markers[target_index].color.b = 1.0f;
                marker_array.markers[target_index].color.a = 0.5f;
              }
              pub_myT_clusters_.publish(marker_array);
            }
            ROS_INFO("my_towel (src: %zu, cluster: %zu, ok_cluster: %zu)",
            msg->size(), cluster_indices.size(), ok);
      }
      catch (std::exception &e)
      {
        ROS_ERROR("%s", e.what());
      }
      now_target_1 = false;
    }

    else{
      ROS_INFO("no data");
    }
  }

  //-----------------------------------------------------------------------
  //自チームのバスタオル以外への処理
  void cb_others(const PointCloud::ConstPtr &msg)
  {
    if (now_target_2==true) {
      visualization_msgs::Marker target_towel = target_towel_ptr;

      try
      {
        std::string frame_id = msg->header.frame_id;
        PointCloud::ConstPtr cloud_src = msg;
        if (target_frame_.empty() == false) //データが入っているとき
        {
          frame_id = target_frame_;
          if (pcl_ros::transformPointCloud(
                  target_frame_, *msg, *cloud_tranformed_others, tf_listener_) == false)
          {
            ROS_ERROR("Failed pcl_ros::transformPointCloud. target_frame = %s",
                      target_frame_.c_str());
            return;
          }
          pub_transformed_.publish(cloud_tranformed_others);
          cloud_src = cloud_tranformed_others;
        }

        //以下 PassThroughフィルタ
        pass_x.setFilterLimits(target_towel.pose.position.x - Catch_width/2,target_towel.pose.position.x + Catch_width/2);  // 0.3 ～ 0.85 m の間にある点群を抽出
        pass_y.setFilterLimits(target_towel.pose.position.y - target_towel.scale.y/2, target_towel.pose.position.y + target_towel.scale.y/2);  // 0.3 ～ 0.85 m の間にある点群を抽出
        // pass_x.setFilterLimits(0,1.5);
        // pass_y.setFilterLimits(0,1.5);

        pass_x.setInputCloud(cloud_src);
        pass_x.filter(*cloud_passthrough_others);
        pass_y.setInputCloud(cloud_passthrough_others);
        pass_y.filter(*cloud_passthrough_others);
        pub_passthrough_others.publish(cloud_passthrough_others);


        //以下クラスタリング
        std::vector<pcl::PointIndices> cluster_indices;
        tree_->setInputCloud(cloud_passthrough_others);
        ec_others.setInputCloud(cloud_passthrough_others);
        ec_others.extract(cluster_indices);
        visualization_msgs::MarkerArray marker_array;
        int target_index = -1;
        int marker_id = 0;
        size_t ok = 0;
        for(std::vector<pcl::PointIndices>::const_iterator
                  it = cluster_indices.begin(),
                  it_end = cluster_indices.end();
                it != it_end; ++it, ++marker_id)
        {
          Eigen::Vector4f min_pt, max_pt;
          pcl::getMinMax3D(*cloud_passthrough_others, *it, min_pt, max_pt);
          Eigen::Vector4f cluster_size = max_pt - min_pt;
          if(cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z()>0)
          {
            visualization_msgs::Marker marker = makeMarker(frame_id, "others", marker_id, min_pt, max_pt, 1.0f, 0.0f, 0.0f, 0.5f);
            marker_array.markers.push_back(marker);
          }
        }
        if(marker_array.markers.empty() == false)
        {
          pub_others_clusters_.publish(marker_array);
        }
        ROS_INFO("others (src: %zu, cluster: %zu, ok_cluster: %zu)",
                  msg->size(), cluster_indices.size(), ok);
      }
      catch (std::exception &e)
      {
        ROS_ERROR("%s", e.what());
      }
      now_target_2 = false;
    }

    else{
      ROS_INFO("no data");
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
    std::string topic_name,topic_name_2;
    pnh_.param("target_frame", target_frame_, std::string("base_link"));
    pnh_.param("topic_name", topic_name, std::string("/my_towels"));
    pnh_.param("topic_name_2", topic_name_2, std::string("/others"));
    ROS_INFO("target_frame = '%s'", target_frame_.c_str());
    ROS_INFO("topic_name = '%s'", topic_name.c_str());
    sub_myT = nh_.subscribe(topic_name, 5, &ClusteringNode::cb_myT, this);
    sub_others = nh_.subscribe(topic_name_2, 5, &ClusteringNode::cb_others, this);
    sub_cluster = nh_.subscribe("myT_clusters", 5, &ClusteringNode::cb_cluster, this);
    pub_transformed_ = nh_.advertise<PointCloud>("cloud_transformed", 1);
    cloud_tranformed_myT.reset(new PointCloud());
    cloud_tranformed_others.reset(new PointCloud());

    // PassThroughフィルタ
    pass_x.setFilterFieldName("x");  // X軸（前方）の値でフィルタをかける
    pass_y.setFilterFieldName("y");  // Y軸（前方）の値でフィルタをかける
    cloud_passthrough_myT.reset(new PointCloud());
    cloud_passthrough_others.reset(new PointCloud());
    pub_passthrough_myT = nh_.advertise<PointCloud>("passthrough_myT", 1);
    pub_passthrough_others = nh_.advertise<PointCloud>("passthrough_others", 1);

    // VoxcelGridフィルタ
    //voxel_.setLeafSize(0.02f, 0.02f, 0.02f); //0.015m間隔でダウンサンプリング
  //  cloud_voxel_.reset(new PointCloud());
    //pub_voxel_ = nh_.advertise<PointCloud>("voxel", 1);

    //以下を追記　クラスタリング
    int catch_min_myT = 100,catch_min_others = 100;
    pnh_.getParam("catch_min_myT", catch_min_myT);
    pnh_.getParam("catch_min_others", catch_min_others);
    // double cut_scale = 0.4;
    // pnh_.getParam("cut_scale", cut_scale);

    tree_.reset(new pcl::search::KdTree<PointT>());
    ec_myT.setClusterTolerance(0.01);//15cm以上離れていれば別のクラスタだとみなす
    ec_myT.setMinClusterSize(catch_min_myT);     //クラスタを構成する点の数(min)
    ec_myT.setMaxClusterSize(300000);  //クラスタを構成する点の数(max)
    ec_myT.setSearchMethod(tree_);   //ある点とクラスタを形成可能な点の探索方法としてKD木を使用する。
    ec_others.setClusterTolerance(0.01);//15cm以上離れていれば別のクラスタだとみなす
    ec_others.setMinClusterSize(catch_min_others);     //クラスタを構成する点の数(min)
    ec_others.setMaxClusterSize(300000);  //クラスタを構成する点の数(max)
    ec_others.setSearchMethod(tree_);   //ある点とクラスタを形成可能な点の探索方法としてKD木を使用する。
    pub_myT_clusters_ = nh_.advertise<visualization_msgs::MarkerArray>("catch_myT_clusters", 1);
    pub_others_clusters_ = nh_.advertise<visualization_msgs::MarkerArray>("catch_others_clusters", 1);
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "catch_clustering");

  // ros::param::set("catch_min_myT", 100);
  // ros::param::set("catch_min_others", 100);
  // ros::param::set("cut_scale", 0.4);

  ClusteringNode pointcloud_test;
  ROS_INFO("Hello Point Cloud!");
  ros::spin();
}
