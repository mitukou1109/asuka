#include "ros/ros.h"
#include "cmath"
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/common.h>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Bool.h"

enum MODE {Move=0, Pull, Catch, Ignore, Remove};
//Move:目的地に迎う（把持は開く）
//Pull:かきこみで回収
//Catch:把持で回収
//Ignore:回収できないから無視して横に進む
//Remove:相手のタオルを把持する

visualization_msgs::MarkerArray others_marker_array_ptr;
visualization_msgs::Marker target_towel_ptr,target_others_ptr;
geometry_msgs::Vector3 catch_destination;
bool now_others=false, now_target = false,now_destination=false,catched=false;

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
  // ROS_INFO("myT_clusters: %zu", msg.markers.size());
}

void cb_others(const visualization_msgs::MarkerArray &msg)
{
  others_marker_array_ptr = msg;
  visualization_msgs::MarkerArray::_markers_type::const_iterator it,it_end;
  for (it = msg.markers.begin(),it_end = msg.markers.end();
       it != it_end; ++it)
  {
      const visualization_msgs::Marker &marker = *it;
      if (marker.ns == "nearest_others")
      {
        target_others_ptr = marker;
      }
  }
  now_others=true;
}

void cb_destination(const geometry_msgs::Vector3 catch_destination_data)
{
  catch_destination = catch_destination_data;
  now_destination= true;
  ROS_INFO("get destination");

}

void cb_catched(const std_msgs::Bool data_catched)
{
  catched = data_catched.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "judge_area_2");
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");

  ros::Subscriber sub_myT = nh.subscribe("myT_clusters",10, cbmyT);
  ros::Subscriber sub_others = nh.subscribe("others_clusters",10, cb_others);
  ros::Subscriber sub_destination = nh.subscribe("catch_destination",10, cb_destination);
  ros::Subscriber sub_catched = nh.subscribe("catched",5, cb_catched);

  ros::Publisher pub = nh.advertise<geometry_msgs::Vector3> ("destination", 10);
  ros::Publisher pub_arm = nh.advertise<visualization_msgs::Marker>("arm_point", 10);


  // ros::param::set("loop_rate", 20);
  // ros::param::set("arm_width", 0.05);//アームの横幅
  // ros::param::set("catch_width", 0.1);//把持に必要な前後幅
  // ros::param::set("ok_threshold", 3);
  double ok_threshold=3;
  double loop_rate = 20;
  double catch_width =0.1;
  double arm_width = 0.05;
  double tolerance = 20;//[mm]
  pn.getParam("arm_width", arm_width);
  pn.getParam("ok_threshold", ok_threshold);
  pn.getParam("loop_rate", loop_rate);
  pn.getParam("catch_width", catch_width);
  pn.getParam("tolerance", tolerance);
  tolerance = tolerance * 0.001;//[mm] to [m]


  int ok_count=0, ok_catch_count=0;
  bool ok;

  double limit_y_min,limit_y_max,limit_x_max,limit_x_min;
  geometry_msgs::Vector3 destination;
  visualization_msgs::Marker target_towel, target_others, arm_point;
  visualization_msgs::MarkerArray others_marker_array;

  ros::Rate loop_rate_(loop_rate);
  while(ros::ok())
  {
    //自チームのタオル以外のデータがあれば保存
    if (now_others==true) {
      others_marker_array = others_marker_array_ptr;
      // target_others = target_others_ptr;
      now_others=false;
      // ROS_INFO("load the data ");
    }
    else{
      others_marker_array.markers.clear();
    }

    //自チームのタオルがあるか
    if(now_target == true)
    {
      // int risk_level = 0;
      target_towel = target_towel_ptr;

      ROS_WARN("Catch towel?");

      while(now_destination ==false) {
        ROS_WARN("wait catch_destination data");
        ros::spinOnce();
      }
      destination=catch_destination;
      double scale_y = destination.z;
      destination.z=(double)Move;

      //把持可能な領域がない場合
      if (scale_y == -1) {
        //何も把持していない場合
        if (catched == false) {
          if (others_marker_array.markers.empty() == false) {
            destination.x = target_others_ptr.pose.position.x;
            destination.y = target_others_ptr.pose.position.y;
            destination.z = (double)Move;

            //相手のタオルを獲得できる領域にいる場合、把持、そうでなければそこまで移動
            if (fabs(target_others_ptr.pose.position.y) < tolerance) {
              destination.z = (double)Remove;
            }
          }
          //何もない場合
          else{
            ROS_INFO("No thing. Move!");
            destination.z = (double)Ignore;
          }
        }

        //すでに相手チームのタオルを把持している場合（誤認に注意、色の指定をしたほうが良いかも）
        else{
          destination.z = (double)Ignore;
        }
      }

      //把持可能な領域がある場合
      else
      {
        ROS_INFO("I can catch towel at x:%f y:%f scale.y:%f",destination.x, destination.y, scale_y*2);

        geometry_msgs::Vector3 pose ,scale;
        pose.x = destination.x; pose.y = destination.y; pose.z = target_towel.pose.position.z;
        scale.y = 2*scale_y; scale.x = 0.15; scale.z = 0.5;
        arm_point = makeMarker("base_link", "arm_point", 0, pose, scale, 0.0, 1.0, 1.0, 0.5);

        if ((arm_width/2 < destination.y+scale_y) && (-1*arm_width/2 > destination.y-scale_y)) {  //回収可能領域に入っているとき
          ok_catch_count++;
          ROS_INFO("destination OK! count=%d pose=%f",ok_catch_count,destination.y);

          if (ok_catch_count >= ok_threshold) {
            ok=true;
            //ok_count=0;
            destination.z = (double)Catch;
            arm_point.color.g = 0.0;
            arm_point.color.b = 0.0;
            arm_point.color.a = 1.0;
            arm_point.ns="ok";
          }
        }
        else{ ok_catch_count=0;}
      }

      pub.publish(destination);
      pub_arm.publish(arm_point);

      now_target = false;
    }

    //自チームのタオルが見つからない場合
    else
    {
      if (catched == false) {
        if (others_marker_array.markers.empty() == false) {
          destination.x = target_others_ptr.pose.position.x;
          destination.y = target_others_ptr.pose.position.y;
          destination.z = (double)Move;

          //相手のタオルを獲得できる領域にいる場合、把持、そうでなければそこまで移動
          if (fabs(target_towel.pose.position.y) < tolerance) {
            destination.z = (double)Remove;
          }
        }
        //何もない場合
        else{
          ROS_INFO("No thing. Move!");
          destination.z = (double)Ignore;
        }
      }

      //すでに相手チームのタオルを把持している場合（誤認に注意、色の指定をしたほうが良いかも）
      else{
        destination.z = (double)Ignore;
      }

      pub.publish(destination);
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
