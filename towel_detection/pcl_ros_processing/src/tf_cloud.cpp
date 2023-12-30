#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;
//座標系におけるレーザの点群のデータ変数
tf::StampedTransform transform;
std::string base_frame,sensor_frame;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  sensor_msgs::PointCloud2 laserPoints_sensor;
  sensor_msgs::PointCloud2 laserPoints_robot;
  laserPoints_sensor = *cloud_msg;
  pcl_ros::transformPointCloud(base_frame, transform, laserPoints_sensor, laserPoints_robot);
  pub.publish(laserPoints_robot);
  ROS_ERROR("transformed!");
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "tf_cloud");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("tf_cloud", 1);

  // Set ROS param
  // ros::param::set("sensor_frame", std::string("camera_color_optical_frame"));
  // ros::param::set("base_frame", std::string("base_link"));
  ros::param::get("sensor_frame", sensor_frame);
  ros::param::get("base_frame", base_frame);
  tf::TransformListener tflistener;

  // static tf::TransformBroadcaster br; //TransformBroadcasterおぶじぇくとのさくせい
  // tf::Transform transform_br;


  ros::Rate loop_rate(50.0);
  bool loop=false;
  while (loop) {
    try{
      tflistener.lookupTransform(base_frame, sensor_frame, ros::Time(0), transform);
      //引数：①取得したい変換のfrom(この座標系から見る)にあたる座標系、②取得したい変換のtoに当たる座標系(①で指定した座標系から見る座標)、
      //③変換したい時間を指定（ros::Time(0)で最新のtrasformを取得）、④結果を取得したいオブジェクト
      ROS_WARN("I got a transform");
      loop=true;
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
  while (ros::ok()) {
    // transform_br.setOrigin( tf::Vector3(0.0, 0.0, 1.25) );
    // tf::Quaternion q;
    // q.setRPY(-1.57, 0, -2.36);
    // //ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
    // transform.setRotation(q); //回転をセット
    // br.sendTransform(tf::StampedTransform(transform_br, ros::Time::now(), sensor_frame ,base_frame));
    // //↑座標変換、ｔｆ：：StampedTransformで情報をまとめて送信（引数：①transformそのもの、②現在の時間（タイムスタンプとして）、③親の座標系の名前、④子（変換対象）の座標系の名前）

    // Spin
    ROS_ERROR("spinOnce");

    ros::spinOnce();
    loop_rate.sleep();
  }
}
