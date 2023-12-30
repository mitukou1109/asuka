#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <geometry_msgs/Vector3.h>
#include "serial/serial.h"

using std::exception;
using std::cout;
using std::cerr;
using std::endl;

//通信設定
const std::string port = "/dev/serialUSB";
const uint32_t baudrate=115200;
serial::Timeout timeout(serial::Timeout::max(), 100,0,0,0);//read:100ms, write:0ms
serial::Serial ser;


// serial::Serial ser;//(port, baudrate,timeout);

//変数定義
geometry_msgs::Vector3 now_destination;

ros::Time last_time;
bool has_flushed_recieve_buffer = false;
bool now=false;

ros::Publisher pub_send;

//コールバック関数--------------------------------------------
void cb_destination(const geometry_msgs::Vector3& destination)
{
  now_destination = destination;
  now = true;
}

//--------------------------------------------------------
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "send_data");
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");

  pub_send = nh.advertise<std_msgs::Bool> ("no_send",10);
  ros::Publisher pub_omlette = nh.advertise<std_msgs::Bool> ("omelette",10);

  ros::Subscriber sub_destination = nh.subscribe("destination",10,cb_destination);
  // ros::Subscriber sub_request = nh.subscribe("send",10,cb_request);

  std::string send_data = "n\n";
  std_msgs::Bool removed, omelette_flag;

  //param
  int loop_rate,plus,request_timeout;
  pn.param("loop_rate",loop_rate,50);
  pn.param("plus",plus,160);
  pn.param("request_timeout",request_timeout,10);

  ser.setPort(port);
  ser.setTimeout(timeout);
  ser.setBaudrate(baudrate);

  //接続待ち
  while (not ser.isOpen()) {
    try{
      ser.open();
    } catch (exception &e) {
      ROS_WARN("can't conect");
      ros::Duration(0.5).sleep();
    }
  }

  ros::Rate loop_rate_(loop_rate);
  while (ros::ok()) {
    ros::Time now_time = ros::Time::now();
    if (ser.getCTS()) {
      if(not has_flushed_recieve_buffer){
        ser.flushInput();
        has_flushed_recieve_buffer = true;
      }

      if (now)
      {
        if (now_destination.z == 0)
        {
          send_data = "x" + std::to_string(int(now_destination.y * 1000)) + "\n";
          ROS_INFO("move destination x:%f", now_destination.y * 1000);
          removed.data = false;
        }

        else if (now_destination.z == 1)
        {
          send_data = "p" + std::to_string(int(now_destination.x * 1000) + plus) + "\n";
          ROS_INFO("Pull down destination y:%f", now_destination.x * 1000);
          removed.data = false;
        }

        else if (now_destination.z == 2)
        {
          send_data = "c" + std::to_string(int(now_destination.x * 1000) + plus) + "\n";
          ROS_INFO("Catch destination y:%f", now_destination.x * 1000);
          removed.data = false;
        }

        else if (now_destination.z == 3)
        {
          send_data = "n\n";
          ROS_INFO("Ignore!");
          removed.data = false;
        }

        else if (now_destination.z == 4)
        {
          send_data = "r" + std::to_string(int(now_destination.x * 1000) + plus + 30) + "\n";
          ROS_INFO("Remove!");
          removed.data = true;
        }

        else
        {
          ROS_INFO("send error (destination.z:%f is not available", now_destination.z);
          send_data = "n\n";
          removed.data = false;
        }
        now = false;
        ser.write(send_data);
        ROS_INFO_STREAM(send_data);
        //pub_send.publish(removed);
        last_time = ros::Time::now();
      }
      
    }

    //cts==false
    else if ((now_time-last_time).toSec() > request_timeout) {
      has_flushed_recieve_buffer = false;
      if (ser.available())
      {
        std::string mode = ser.readline(); //65536,"\n"

        if (mode == "o\n")
        {
          ROS_INFO("Execute Omelette");
          omelette_flag.data = true;
        }

        else if (mode == "n\n")
        {
          ROS_INFO("Not Execute Omelette");
          omelette_flag.data = false;
        }

        else
        {
          ROS_ERROR("mode error");
          omelette_flag.data = false;
        }
        pub_omlette.publish(omelette_flag);
      }
    }

    loop_rate_.sleep();
    ros::spinOnce();
  }
}
