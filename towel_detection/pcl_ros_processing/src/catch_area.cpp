#include "ros/ros.h"
#include "cmath"
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/common.h>
#include "geometry_msgs/Vector3.h"

visualization_msgs::MarkerArray others_marker_array_ptr;
visualization_msgs::Marker target_towel_ptr;
bool now_others=false, now_target = false;

visualization_msgs::Marker makeMarker(
    const std::string &frame_id, const std::string &marker_ns,
    int marker_id,
    geometry_msgs::Vector3 pose, geometry_msgs::Vector3 scale,
    float r, float g, float b, float a);

void cbmyT(const visualization_msgs::MarkerArray &msg)
{

  const visualization_msgs::Marker *target = NULL;
  visualization_msgs::MarkerArray::_markers_type::const_iterator it,it_end;
  for (it = msg.markers.begin(),it_end = msg.markers.end();
       it != it_end; ++it)
  {
      const visualization_msgs::Marker &marker = *it;
      if (marker.ns == "nearest_towel")
      {
        target_towel_ptr = marker;
        // target_towel_ptr = *target;
        now_target = true;
      }
  }
  ROS_INFO("catch_myT_clusters: %zu", msg.markers.size());
}

void cb_others(const visualization_msgs::MarkerArray &msg)
{
  others_marker_array_ptr = msg;
  now_others=true;
  ROS_INFO("catch_others_clusters: %zu", msg.markers.size());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "catch_area");
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");

  ros::Subscriber sub_myT = nh.subscribe("catch_myT_clusters",10, cbmyT);
  ros::Subscriber sub_others = nh.subscribe("catch_others_clusters",10, cb_others);

  ros::Publisher pub = nh.advertise<geometry_msgs::Vector3> ("catch_destination", 10);
  // ros::Publisher pub_arm = nh.advertise<visualization_msgs::Marker>("arm_point", 10);


  // ros::param::set("loop_rate", 15);
  // ros::param::set("arm_width", 0.05);
  // ros::param::set("ok_threshold", 3);
  double ok_threshold = 3, loop_rate = 20;
  pn.getParam("ok_threshold", ok_threshold);
  pn.getParam("loop_rate", loop_rate);
  double arm_width;
  pn.getParam("arm_width", arm_width);



  int ok_count=0;
  bool ok;

  double limit_min,limit_max;
  geometry_msgs::Vector3 destination;
  visualization_msgs::Marker target_towel, arm_point;
  visualization_msgs::MarkerArray others_marker_array;

  ros::Rate loop_rate_(loop_rate);
  while(ros::ok())
  {
    // ROS_INFO("roop start ");
    if(now_target == true)
    {
      ROS_INFO("----------------------------- ");
      // int risk_level = 0;
      target_towel = target_towel_ptr;
      ROS_INFO("000000000000000000000000000000 ");
      if (now_others==true) {
        others_marker_array = others_marker_array_ptr;
        now_others=false;
        // ROS_INFO("load the data ");
      }
      else{
        others_marker_array.markers.clear();
      }

      limit_min= target_towel.pose.position.y - target_towel.scale.y /2;
      limit_max= target_towel.pose.position.y + target_towel.scale.y /2;
      ROS_INFO("first limit_min:%f limit_max:%f", limit_min,limit_max);

      ROS_INFO("others_marker_array: %zu", others_marker_array.markers.size());
      for (size_t i = 0; i < others_marker_array.markers.size(); i++) //othersクラスタ（障害物）の影響を確かめる
      {
        double y = others_marker_array.markers[i].pose.position.y;
        if ((y > limit_min) && (y < limit_max)) //範囲内にクラスタの中心がある場合(2個以上ないと仮定)
        {
          double end_y_right = y + others_marker_array.markers[i].scale.y/2;
          double end_y_left = y - others_marker_array.markers[i].scale.y/2;
          ROS_INFO("change area? r:%f or l:%f", end_y_right,end_y_left);

          double dr = limit_max - end_y_right;
          double dl = end_y_left - limit_min;
          if ((dr<=0)||(dl<=0)) { //端が範囲から出ている場合
            if (dr<=0) { limit_max = end_y_left;ROS_INFO("changed_right!");}
            if (dl<=0) { limit_min = end_y_right;ROS_INFO("changed left!");}
          }
          //以下、範囲内部に完全に収まっている場合
          else if (dr >= dl) //領域が広い方を優先して用いる
          {
            ROS_INFO("choose area limit_min: %f -> %f", limit_min,end_y_right);
            limit_min = end_y_right;
          }

          else //dr < dl
          {
            ROS_INFO("choose area limit_max: %f -> %f", limit_max,end_y_left);
            limit_max = end_y_left;
          }
        }

        else if (y > limit_max) //クラスタ中心が範囲の右側の外にある場合
        {
          double end_y = y - others_marker_array.markers[i].scale.y/2;
          ROS_INFO("change limit_max?: %f -> %f", limit_max,end_y);
          if (end_y < limit_max)
          {  //クラスタの左端が領域内に入っている場合、領域を狭める
            ROS_INFO("changed!");
            limit_max = end_y;
          }
        }

        else if (y < limit_min) //クラスタ中心が範囲の左側の外にある場合
        {
          double end_y = y + others_marker_array.markers[i].scale.y/2;
          ROS_INFO("change limit_min?: %f -> %f", limit_min,end_y);
          if (end_y > limit_min)
          {  //クラスタの右端が領域内に入っている場合、領域を狭める
            ROS_INFO("changed");
            limit_min = end_y;
          }
        }
      }

      ROS_WARN("result limit_min:%f limit_max:%f", limit_min,limit_max);
      if ((limit_max - limit_min) > arm_width)
      {
        destination.x = target_towel.pose.position.x;
        destination.y = (limit_max + limit_min)/2;  //中間地点
        destination.z = (limit_max - limit_min)/2;  //scale.y/2 ｚの位置はいらないので幅の格納用として使用
        ROS_INFO("I can get towel at x:%f",destination.y);

        // std::string frame_id = target_towel.header.frame_id;
        geometry_msgs::Vector3 pose ,scale;
        pose.x = destination.x; pose.y = destination.y; pose.z = target_towel.pose.position.z;
        scale.y = limit_max - limit_min; scale.x = 1.0; scale.z = 0.5;
        arm_point = makeMarker("base_link", "arm_point", 0, pose, scale, 1.0, 1.0, 0.0, 0.5);

        pub.publish(destination);
        // pub_arm.publish(arm_point);

      }

      else
      {
        ROS_ERROR("I can't do easy.");
        // destination.x = target_towel.pose.position.x;
        // destination.y = target_towel.pose.position.y;  //中間地点
        destination.z = -1;
        pub.publish(destination);
        arm_point.color.b = 0.0;
        arm_point.ns="no";
        ok_count=0;
      }

      now_target = false;
    }

    else{
      ROS_INFO("Move!");
      ok_count=0;
    }

    ros::spinOnce();
    loop_rate_.sleep();
  }

}

visualization_msgs::Marker makeMarker(
    const std::string &frame_id, const std::string &marker_ns,
    int marker_id,
    geometry_msgs::Vector3 pose, geometry_msgs::Vector3 scale,
    float r, float g, float b, float a)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = marker_ns;
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pose.x;
  marker.pose.position.y = pose.y;
  marker.pose.position.z = pose.z;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = scale.x;
  marker.scale.y = scale.y;
  marker.scale.z = scale.z;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  marker.lifetime = ros::Duration(0.3);
  return marker;
}
