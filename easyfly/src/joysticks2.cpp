#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyradio.h>
#include <crazyflie_cpp/Crazyflie.h>
#include <boost/bind.hpp>
/******************

MUST RUN IN ROOT!!!!!!!!!!!!!!


*************************/
std::string g_uri;
class fly212
{
public:
  fly212(int argc, char **argv);
  void run(double frequency);
  std::string uri;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy, int uav);
  ros::NodeHandle nh;

  ros::Subscriber joy_sub0;
  ros::Subscriber joy_sub1;
//  int init_scan(int argc, char **argv);
//  int connect_get_param(int argc, char **argv);
  void iteration(const ros::TimerEvent& e);
  
  float pitch_sp[2];
  float roll_sp[2];
  float yaw_sp[2];
  float thrust_sp[2];
//  Crazyflie cf;

};

fly212::fly212(int argc, char **argv)//:
//cf(g_uri)
{
//  uri=g_uri;
//  Crazyflie cf(uri);
//  cf(uri);
//  cf.requestParamToc();
 //  joy_sub0 = nh.subscribe<sensor_msgs::Joy>("/joygroup0/joy",5,&fly212::joyCallback,this);
 //  joy_sub1 = nh.subscribe<sensor_msgs::Joy>("/joygroup1/joy",5,&fly212::joyCallback,this);
  joy_sub0 = nh.subscribe<sensor_msgs::Joy>
    ("/joygroup0/joy",5,boost::bind(&fly212::joyCallback, this, _1, 0));
  joy_sub1 = nh.subscribe<sensor_msgs::Joy>
    ("/joygroup1/joy",5,boost::bind(&fly212::joyCallback, this, _1, 1));
}
void fly212::run(double frequency)
{
  ros::NodeHandle node;
  ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &fly212::iteration, this);
  ros::spin();
}
void fly212::iteration(const ros::TimerEvent& e)
{
  float dt = e.current_real.toSec() - e.last_real.toSec();
  printf("pitch: %f\t", pitch_sp[0]);
  printf("pitch: %f\n", pitch_sp[1]);

  printf("roll: %f\t", roll_sp[0]);
  printf("roll: %f\n", roll_sp[1]);

  printf("yaw: %f\t", yaw_sp[0]);
  printf("yaw: %f\n", yaw_sp[1]);

  printf("thrust: %f\t", thrust_sp[0]);
  printf("thrust: %f\n", thrust_sp[1]);

//  cf.sendSetpoint(-roll_sp[0]*20, -pitch_sp[0]*20, -yaw_sp[0]*50, thrust_sp[0]*40000);
}
void fly212::joyCallback(const sensor_msgs::Joy::ConstPtr& joy, int uav)
{
  if(uav==0){
  pitch_sp[0] = joy->axes[5];
  roll_sp[0] = joy->axes[2];
  yaw_sp[0] = joy->axes[0];
  thrust_sp[0] = joy->axes[1];
  if(thrust_sp[0]<0)
    thrust_sp[0]=0;
  }
  else if (uav==1){

  pitch_sp[1] = joy->axes[5];
  roll_sp[1] = joy->axes[2];
  yaw_sp[1] = joy->axes[0];
  thrust_sp[1] = joy->axes[1];
  if(thrust_sp[1]<0)
    thrust_sp[1]=0;
  }
}

int main(int argc, char **argv)
{
//  int ret = init_scan(argc, argv);
  ros::init(argc, argv, "joysticks2");
  
  fly212 Fly212(argc, argv);
  Fly212.run(10);


  return 0;


}