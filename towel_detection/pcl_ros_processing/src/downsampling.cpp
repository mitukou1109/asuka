#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>  //PassThroughフィルタ

ros::Publisher pub;
ros::Publisher pub_passthrough_;
double down_rate=0.005;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  /*以下を追記　PassThroughフィルタ
  PassThroughフィルタ：得られた点群のうち、一定の範囲内にある点群のみを抽出する*/
  // pcl::PassThrough<pcl::PCLPointCloud2> pass_; //PassThroughフィルタの宣言
  // pcl::PCLPointCloud2::Ptr cloud_passthrough_;
  //
  // //  PassThroughフィルタ　設定
  // pass_.setFilterFieldName("y");  // z軸（前方）の値でフィルタをかける
  // pass_.setFilterLimits(width_min, width_max);  // 間にある点群を抽出
  // cloud_passthrough_.reset(new pcl::PCLPointCloud2());
  //
  // //以下 PassThroughフィルタ
  // pass_.setInputCloud(cloudPtr);
  // pass_.filter(*cloud_passthrough_);
  // pub_passthrough_.publish(cloud_passthrough_);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);

  // sor.setLeafSize (0.1, 0.1, 0.1);
  sor.setLeafSize (down_rate, down_rate, down_rate);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "downsampling");
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");


  // Set ROS param
  // ros::param::set("down_rate", 0.005);
  // ros::param::set("width_min", -0.5);
  // ros::param::set("width_max", 0.5);

  // double width_max ,width_min;
  // pn.getParam("width_max", width_max);
  // pn.getParam("width_min", width_min);
  pn.getParam("down_rate", down_rate);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("downsampling", 1);
  pub_passthrough_ = nh.advertise<pcl::PCLPointCloud2>("passthrough", 1);



  // Spin
  ros::spin ();
}
