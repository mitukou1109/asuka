#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;
double dist_th = 0.01;
int loop = 1;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::fromROSMsg (*input, cloud);



  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;



  for (size_t i = 0; i < loop; i++) {

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);



    seg.setDistanceThreshold (dist_th);

    seg.setInputCloud (cloud.makeShared());
    seg.segment (*inliers, *coefficients);

    for (size_t i = 0; i < cloud.points.size(); ++i){
      cloud.points[i].a=255;
    }

    if(inliers->indices.size() == 0){
      std::cerr << "Could not estimate a planar model for the given data" << std::endl;
    }
    else{//平面の場合の処理
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

      pcl::copyPointCloud(cloud, *cloud_output);

      extract.setInputCloud(cloud_output);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(cloud);
      /*    for (size_t i = 0; i < inliers->indices.size (); ++i) {
      // std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
      // << cloud.points[inliers->indices[i]].y << " "
      // << cloud.points[inliers->indices[i]].z << std::endl;
      cloud.points[inliers->indices[i]].r = 255;
      cloud.points[inliers->indices[i]].g = 255;
      cloud.points[inliers->indices[i]].b = 255;
      cloud.points[inliers->indices[i]].a = 0;
    }*/
  }

  /*  for(size_t i = 0; i < cloud.points.size(); ++i){
  if (cloud.points[i].a!=0) {
  cloud_output.push_back(cloud[i]);
}*/
  }

  pub.publish(cloud);

  // save pcd file
  //pcl::io::savePCDFileASCII ("save.pcd", cloud);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "planar_segmentation");
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");

  // Set ROS param
  // ros::param::set("dist_th", 0.02);
  // ros::param::set("loop", 1);
  pn.getParam("loop", loop);
  pn.getParam("dist_th", dist_th);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the model coefficients
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> > ("planar_segmentation", 1);
  // Spin
  ros::spin ();
}
