#include "ros/ros.h"
#include "cmath"
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/common.h>
#include "geometry_msgs/Vector3.h"

enum MODE {Move=0, Pull, Catch, Ignore, Remove};
//Move:目的地に迎う
//Pull:かきこみで回収
//Catch:把持で回収
//Ignore:回収できないから無視して横に進む

visualization_msgs::MarkerArray others_marker_array_ptr;
visualization_msgs::Marker target_towel_ptr,target_others_ptr;
geometry_msgs::Vector3 catch_destination;
bool now_others=false, now_target = false,now_destination=false;

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
  // ROS_INFO("others_clusters: %zu", msg.markers.size());
}

void cb_destination(const geometry_msgs::Vector3 catch_destination_data)
{
  catch_destination = catch_destination_data;
  now_destination= true;
  ROS_INFO("get destination");

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "judge_area_2");
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");

  ros::Subscriber sub_myT = nh.subscribe("myT_clusters",10, cbmyT);
  ros::Subscriber sub_others = nh.subscribe("others_clusters",10, cb_others);
  ros::Subscriber sub_destination = nh.subscribe("catch_destination",10, cb_destination);

  ros::Publisher pub = nh.advertise<geometry_msgs::Vector3> ("destination", 10);
  ros::Publisher pub_arm = nh.advertise<visualization_msgs::Marker>("arm_point", 10);


  // ros::param::set("loop_rate", 15);
  // ros::param::set("arm_width", 0.05);//アームの横幅
  // ros::param::set("catch_width", 0.1);//把持に必要な前後幅
  // ros::param::set("ok_threshold", 3);
  double ok_threshold = 3;
  double loop_rate =15;
  double catch_width = 0.1;
  double arm_width = 0.05;
  pn.getParam("arm_width", arm_width);
  pn.getParam("ok_threshold", ok_threshold);
  pn.getParam("loop_rate", loop_rate);
  pn.getParam("catch_width", catch_width);
  double tolerance = 20;//[mm]
  pn.getParam("tolerance", tolerance);
  tolerance = tolerance * 0.001;//[mm] to [m]
  double pull_space = 30;//[mm]
  pn.getParam("pull_space", pull_space);
  pull_space=pull_space*0.001;

  int ok_count=0, ok_catch_count=0,ok_remove_count = 0;
  bool ok;

  double limit_y_min,limit_y_max,limit_x_max,limit_x_min;
  geometry_msgs::Vector3 destination;
  visualization_msgs::Marker target_towel, arm_point;
  visualization_msgs::MarkerArray others_marker_array;

  ros::Rate loop_rate_(loop_rate);
  while(ros::ok())
  {
    // ROS_INFO("roop start ");
    // for (size_t i = 0; i < 3; i++) { ros::spinOnce(); }
    if(now_target == true)
    {
      // int risk_level = 0;
      target_towel = target_towel_ptr;
      now_target = false;
      if (now_others==true) {
        others_marker_array = others_marker_array_ptr;
        now_others=false;
        // ROS_INFO("load the data ");
      }
      else{
        others_marker_array.markers.clear();
        target_others_ptr.ns="NULL";
      }

      limit_y_min= target_towel.pose.position.y - target_towel.scale.y /2;
      limit_y_max= target_towel.pose.position.y + target_towel.scale.y /2;
      limit_x_min= target_towel.pose.position.x - target_towel.scale.x /2;
      limit_x_max= target_towel.pose.position.x + target_towel.scale.x /2;
      ROS_INFO("limit_y_min:%f limit_y_max:%f", limit_y_min,limit_y_max);

      // ROS_INFO("others_marker_array: %zu", others_marker_array.markers.size());

      //------------------------------------------------------------------------------
      //以下、回収可能領域の検出
      for (size_t i = 0; i < others_marker_array.markers.size(); i++) //othersクラスタ（障害物）の影響を確かめる
      {
        double x = others_marker_array.markers[i].pose.position.x;
        double y = others_marker_array.markers[i].pose.position.y;

        //自チームタオルよりも向こう側にある場合、それは完全に無視する
        if ((x - others_marker_array.markers[i].scale.x/2) > limit_x_max)
        {
          // ROS_WARN("throw out");
          continue;
        }

        else if((x - others_marker_array.markers[i].scale.x/2) > (target_towel.pose.position.x + target_towel.scale.x /2 - pull_space))
        {
          limit_x_max = x - others_marker_array.markers[i].scale.x/2;
          continue;
        }

        others_marker_array.markers[i].ns = "danger";

        if ((y > limit_y_min) && (y < limit_y_max)) //範囲内にクラスタの中心がある場合(2個以上ないと仮定)
        {
          double end_y_right = y + others_marker_array.markers[i].scale.y/2;
          double end_y_left = y - others_marker_array.markers[i].scale.y/2;
          // ROS_ERROR("change area? r:%f or l:%f", end_y_right,end_y_left);

          double dr = limit_y_max - end_y_right;
          double dl = end_y_left - limit_y_min;
          if ((dr<=0)||(dl<=0)) { //端が範囲から出ている場合
            if (dr<=0) { limit_y_max = end_y_left;ROS_ERROR("changed_right!");}
            if (dl<=0) { limit_y_min = end_y_right;ROS_ERROR("changed left!");}
          }
          //以下、範囲内部に完全に収まっている場合
          else if (dr >= dl) //領域が広い方を優先して用いる
          {
            // ROS_WARN("choose area limit_y_min: %f -> %f", limit_y_min,end_y_right);
            limit_y_min = end_y_right;
          }

          else //dr < dl
          {
            // ROS_WARN("choose area limit_y_max: %f -> %f", limit_y_max,end_y_left);
            limit_y_max = end_y_left;
          }
        }

        else if (y > limit_y_max) //クラスタ中心が範囲の右側の外にある場合
        {
          double end_y = y - others_marker_array.markers[i].scale.y/2;
          // ROS_WARN("change limit_y_max?: %f -> %f", limit_y_max,end_y);
          if (end_y < limit_y_max)
          {  //クラスタの左端が領域内に入っている場合、領域を狭める
            // ROS_ERROR("changed!");
            limit_y_max = end_y;
          }
        }

        else if (y < limit_y_min) //クラスタ中心が範囲の左側の外にある場合
        {
          double end_y = y + others_marker_array.markers[i].scale.y/2;
          // ROS_WARN("change limit_y_min?: %f -> %f", limit_y_min,end_y);
          if (end_y > limit_y_min)
          {  //クラスタの右端が領域内に入っている場合、領域を狭める
            // ROS_ERROR("changed");
            limit_y_min = end_y;
          }
        }

        else    //領域外の場合（特に邪魔にならない場合）
        {
          others_marker_array.markers[i].ns = "safe";
        }
      }

      if (limit_y_max - limit_y_min > arm_width)  //かきこみで回収できる場合
      {
        destination.x = limit_x_max;  //タオルの奥を指定
        destination.y = (limit_y_max + limit_y_min)/2;  //中間地点
        destination.z = (double)Move;
        ROS_INFO("I can get towel at y:%f",destination.y);
        ROS_INFO("result limit_y_min:%f limit_y_max:%f", limit_y_min,limit_y_max);

        // std::string frame_id = target_towel.header.frame_id;
        geometry_msgs::Vector3 pose ,scale;
        scale.y = limit_y_max - limit_y_min; scale.x = 1.0; scale.z = 0.5;
        pose.x = destination.x - scale.x/2; pose.y = destination.y; pose.z = target_towel.pose.position.z;
        arm_point = makeMarker("base_link", "arm_point", 0, pose, scale, 1.0, 1.0, 0.0, 0.5);

//        if (fabs(destination.x) < arm_width/2) {
        if (fabs(destination.y) < tolerance) {  //回収可能領域に入っているとき
          ok_count++;
          ROS_INFO("destination OK! pull count=%d pose=%f",ok_count,destination.y);

          if (ok_count >= ok_threshold) {
            ok=true;
            //ok_count=0;
            destination.z = (double)Pull;
            arm_point.color.b = 1.0;
            arm_point.ns="ok";
            ok_catch_count=0;
            ok_remove_count=0;
          }
        }
        else{ ok_count=0;}

        pub.publish(destination);
        pub_arm.publish(arm_point);

      }

      else  //かきこみでは無理な場合（把持モード）
      {
        ok_count=0;
        ROS_WARN("Catch towel?");

//最新のtargey_others_ptrか確認
        if(target_others_ptr.ns!="NULL"){
          //相手のタオルの中心が自分のタオルの下端よりも下にある場合、Removeを行うtarget_others_ptr.scale.x/2+
          if (target_others_ptr.scale.x / 4 + target_others_ptr.pose.position.x < (target_towel.pose.position.x - target_towel.scale.x / 2))
          {
            if(fabs(target_towel.pose.position.y)>0.5){
              destination.y=target_towel.pose.position.y;
              destination.z=(double)Move;
              pub.publish(destination);
            }

            else{
              destination.x = target_others_ptr.pose.position.x;
              destination.y = target_others_ptr.pose.position.y;
              destination.z = (double)Move;

              geometry_msgs::Vector3 pose, scale;
              pose.x = destination.x;
              pose.y = destination.y;
              pose.z = target_others_ptr.pose.position.z;
              scale.y = 2 * tolerance;
              scale.x = target_others_ptr.scale.x;
              scale.z = 0.5;
              arm_point = makeMarker("base_link", "arm_point", 0, pose, scale, 1.0, 1.0, 1.0, 0.5);

              //相手のタオルを獲得できる領域にいる場合、把持、そうでなければそこまで移動
              if (fabs(target_others_ptr.pose.position.y) < tolerance)
              {
                ok_remove_count++;
                ROS_WARN("Remove others! count=%d pose=%f", ok_remove_count, destination.y);

                if (ok_remove_count >= 2)
                {
                  //ok = true;
                  //ok_count=0;
                  destination.z = (double)Remove;
                  arm_point.color.r = 1.0;
                }
                //else{destination.z=(double)Stop;}
              }
              //移動が必要な場合
              else
              {
                ok_remove_count = 0;
              }

              pub_arm.publish(arm_point);
              pub.publish(destination);

            }
            ros::spinOnce();
            loop_rate_.sleep();
            continue;
          }
        }


        //以下、把持モード
        while(now_destination ==false) {
          ROS_WARN("wait catch_destination data");
          ros::spinOnce();
        }
        destination=catch_destination;
        double scale_y = destination.z;
        destination.z=(double)Move;

        //impossible catch
        if (scale_y == -1) {
          ROS_ERROR("I can't catch now");
          if (target_others_ptr.ns != "NULL")
          {
            //相手のタオルの中心が自分のタオルの下端よりも下にある場合、Removeを行うtarget_others_ptr.scale.x/2+
            if (fabs(target_towel.pose.position.y) > 0.5)
            {
              destination.y = target_towel.pose.position.y;
              destination.z = (double)Move;
              pub.publish(destination);
            }

            else
            {
              destination.x = target_others_ptr.pose.position.x;
              destination.y = target_others_ptr.pose.position.y;
              destination.z = (double)Move;

              geometry_msgs::Vector3 pose, scale;
              pose.x = destination.x;
              pose.y = destination.y;
              pose.z = target_others_ptr.pose.position.z;
              scale.y = 2 * tolerance;
              scale.x = target_others_ptr.scale.x;
              scale.z = 0.5;
              arm_point = makeMarker("base_link", "arm_point", 0, pose, scale, 1.0, 1.0, 1.0, 0.5);

              //相手のタオルを獲得できる領域にいる場合、把持、そうでなければそこまで移動
              if (fabs(target_others_ptr.pose.position.y) < tolerance)
              {
                ok_remove_count++;
                ROS_WARN("Remove others! count=%d pose=%f", ok_remove_count, destination.y);

                if (ok_remove_count >= 2)
                {
                  //ok = true;
                  //ok_count=0;
                  destination.z = (double)Remove;
                  arm_point.color.r = 1.0;
                }
                //else{destination.z=(double)Stop;}
              }
              //移動が必要な場合
              else
              {
                ok_remove_count = 0;
              }

              pub_arm.publish(arm_point);
              pub.publish(destination);
            }
            ros::spinOnce();
            loop_rate_.sleep();
            continue;
          }
          // destination.x=target_towel.pose.position.x;
          // destination.y=target_towel.pose.position.y;
          pub.publish(destination);
          destination.z=(double)Ignore;
          ros::spinOnce();
          loop_rate_.sleep();
          continue;
        }

        ROS_INFO("I can catch towel at x:%f y:%f scale.y:%f",destination.x, destination.y, scale_y*2);
        // ROS_INFO("limit_y_min:%f limit_y_max:%f", limit_y_min,limit_y_max);

        // std::string frame_id = target_towel.header.frame_id;
        geometry_msgs::Vector3 pose ,scale;
        pose.x = destination.x; pose.y = destination.y; pose.z = target_towel.pose.position.z;
        scale.y = 2*scale_y; scale.x = 0.15; scale.z = 0.5;
        arm_point = makeMarker("base_link", "arm_point", 0, pose, scale, 0.0, 1.0, 1.0, 0.5);

        if (fabs(destination.y) < tolerance) {  //回収可能領域に入っているとき
          ok_catch_count++;
          ROS_INFO("destination OK! catch count=%d pose=%f",ok_catch_count,destination.y);

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
        else{ ok_catch_count=0; }

        pub.publish(destination);
        pub_arm.publish(arm_point);

      }


    }

    else{
      ROS_INFO("No towel. Move!");
      destination.z = (double)Ignore;
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
