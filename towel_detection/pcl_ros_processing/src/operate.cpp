#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "cmath"
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/common.h>


void cbmyT(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
  const visualization_msgs::Marker *target = NULL;
  visualization_msgs::MarkerArray::_markers_type::const_iterator it,it_end;
  for (it = msg->markers.begin(),it_end = msg->markers.end();
       it != it_end; ++it)
  {
      const visualization_msgs::Marker &marker = *it;
      if (marker.ns == "nearest_towel")
      {
        target = &marker;
      }

      ROS_INFO("clusters: %zu", msg->markers.size());
      if(target != NULL)
      {
        float dx = target->pose.position.x;
        float dy = target->pose.position.y;
        float dz = target->pose.position.z;
        ROS_INFO("target: %f, %f, %f", dx,dy,dz);
      }
  }
}

void cb_others(const visualization_msgs::MarkerArray::ConstPtr &msg)
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "operate");
  ros::NodeHandle nh;

  ros::Subscriber sub_myT = nh.subscribe("myT_clusters",10, cbmyT);
  ros::Subscriber sub_others = nh.subscribe("others_clusters",10, cb_others);

//  ros::Rate loop_rate(10.0);
//  while(ros::ok())
//  {
    ros::spin();

    //loop_rate.sleep();
//  }

}
