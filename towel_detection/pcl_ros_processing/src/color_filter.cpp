#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub_my;
ros::Publisher pub_other;

void
PointXYZRGBAtoXYZHSV (const pcl::PointXYZRGBA& in,
                     pcl::PointXYZHSV&        out);

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Define PointCloud2 data
  pcl::PointCloud<pcl::PointXYZRGBA> in_cloud;
//  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr myT_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr others_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*input, in_cloud);

  // for (size_t i = 0; i < in_cloud.points.size(); ++i){
  //   in_cloud.points[i].a = 255;
  // }

  // convert rgb to hsv
  pcl::PointCloud<pcl::PointXYZHSV> hsv_cloud;
  PointCloudXYZRGBAtoXYZHSV(in_cloud, hsv_cloud);

  // setting alpha = 1.0

  // set frame id
  myT_cloud->header.frame_id = in_cloud.header.frame_id;
  others_cloud->header.frame_id = in_cloud.header.frame_id;
  // std::cerr << "in_cloud frame id=" << in_cloud.header.frame_id << std::endl;

  // color parameter
  int max_r, min_r, max_g, min_g, max_b, min_b;
  int max_h, min_h, max_s, min_s, max_v, min_v;

  /*ros::param::get("max_r", max_r);
  ros::param::get("min_r", min_r);
  ros::param::get("max_g", max_g);
  ros::param::get("min_g", min_g);
  ros::param::get("max_b", max_b);
  ros::param::get("min_b", min_b);*/

  ros::param::get("max_h", max_h);
  ros::param::get("min_h", min_h);
  ros::param::get("max_s", max_s);
  ros::param::get("min_s", min_s);
  ros::param::get("max_v", max_v);
  ros::param::get("min_v", min_v);

  int num=0,sum_h=0,sum_s=0,sum_v=0;

  // color filter（該当する色の点群をまとめる）
  for (size_t i = 0; i < in_cloud.points.size(); ++i){
    //rgbでフィルタ
    /*if (in_cloud.points[i].r > min_r && in_cloud.points[i].r < max_r &&
        in_cloud.points[i].g > min_g && in_cloud.points[i].g < max_g &&
        in_cloud.points[i].b > min_b && in_cloud.points[i].b < max_b
      ){*/
    //hsvでフィルタ
    if (hsv_cloud.points[i].h > min_h && hsv_cloud.points[i].h < max_h &&
        hsv_cloud.points[i].s > min_s && hsv_cloud.points[i].s < max_s &&
        hsv_cloud.points[i].v > min_v && hsv_cloud.points[i].v < max_v
       ){
      // sum_h+=hsv_cloud.points[i].h;
      // sum_s+=hsv_cloud.points[i].s;
      // sum_v+=hsv_cloud.points[i].v;
      // num++;
      inliers->indices.push_back(i);
    }
  }
  // ROS_INFO("h:%d, s:%d, v:%d",sum_h/num,sum_s/num,sum_v/num);
  //色分け
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  pcl::copyPointCloud(in_cloud, *myT_cloud);
  pcl::copyPointCloud(in_cloud, *others_cloud);

  extract.setInputCloud(myT_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false); //自チームの色のものだけを残す
  extract.filter(*myT_cloud);  //→ｍｙT_cloudに保存

  extract.setInputCloud(others_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  //自チーム以外の物だけを残す
  extract.filter(*others_cloud);

  // publish ros msg
  sensor_msgs::PointCloud2 myT_output,others_output;
  if(myT_cloud->size() > 0){
    toROSMsg(*myT_cloud, myT_output);
    pub_my.publish(myT_output);
  }
  if(others_cloud->size() > 0){
    toROSMsg(*others_cloud, others_output);
    pub_other.publish(others_output);
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "color_filter");
  ros::NodeHandle nh;

  /*ros::param::set("max_r", 255);
  ros::param::set("min_r", 0);
  ros::param::set("max_g", 255);
  ros::param::set("min_g", 0);
  ros::param::set("max_b", 255);
  ros::param::set("min_b", 0);*/
  // ros::param::set("max_h", 360);
  // ros::param::set("min_h", 0);
  // ros::param::set("max_s", 255);
  // ros::param::set("min_s", 0);
  // ros::param::set("max_v", 255);
  // ros::param::set("min_v", 0);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the model coefficients
  pub_my = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> > ("my_towels", 1);
  pub_other = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> > ("others", 1);
  // Spin
  ros::spin ();
}

void
PointXYZRGBAtoXYZHSV (const pcl::PointXYZRGBA& in,
                     pcl::PointXYZHSV& out)
 {
 const unsigned char max = std::max (in.r, std::max (in.g, in.b));
 const unsigned char min = std::min (in.r, std::min (in.g, in.b));

 out.x = in.x; out.y = in.y; out.z = in.z;
 out.v = static_cast <float> (max) / 255.f;

 if (max == 0) // division by zero
   {
   out.s = 0.f;
   out.h = 0.f; // h = -1.f;
   return;
 }

 const float diff = static_cast <float> (max - min);
 out.s = diff / static_cast <float> (max);
 if (min == max) // diff == 0 -> division by zero
 {
   out.h = 0;
   return;
 }
 if      (max == in.r) out.h = 60.f * (      static_cast <float> (in.g - in.b) / diff);
 else if (max == in.g) out.h = 60.f * (2.f + static_cast <float> (in.b - in.r) / diff);
 else                  out.h = 60.f * (4.f + static_cast <float> (in.r - in.g) / diff); // max == b
 if (out.h < 0.f) out.h += 360.f;
}
