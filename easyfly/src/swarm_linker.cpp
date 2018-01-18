#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <easyfly/pos_est.h>
#include <easyfly/att_est.h>
#include <easyfly/output.h>
#include <crazyflie_driver/num_vehiclepub.h>
#include "crazyflie_driver/Yaw_est.h"
#include <easyfly/commands.h>
#include <geometry_msgs/TransformStamped.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include "commons.h"
#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyradio.h>
#include <crazyflie_cpp/Crazyflie.h>
int g_vehicle_num; //get from the launch file
class SwarmLinker
{
private:
	std::vector<ros::Publisher> m_pos_estpub_v;
	std::vector<ros::Publisher> m_att_estpub_v;
	std::vector<ros::Subscriber> m_viconsub_v,m_yawsub_v;
	std::vector<easyfly::pos_est> m_pos_est_v;
	std::vector<easyfly::att_est> m_att_est_v;
	std::vector<easyfly::output> m_output_v;
	std::vector<geometry_msgs::Vector3> m_lpos_v;
	std::vector<float> m_lyaw_v;
	std::vector<ros::Time> m_lpos_time_v;
	std::vector<ros::Time> m_lyaw_time_v;
	std::vector<std::string> m_defaultUri_v, m_uri_v;
	//std::vector<Crazyflie> m_cf_v;
	
	//vicon estimator:
	float m_vicon_freq;
	float m_vicon_pos_err;

public:
	SwarmLinker(ros::NodeHandle& nh)
	:m_pos_estpub_v(g_vehicle_num)
	,m_att_estpub_v(g_vehicle_num)
	,m_pos_est_v(g_vehicle_num)
	,m_att_est_v(g_vehicle_num)
	,m_viconsub_v(g_vehicle_num)
	,m_lpos_v(g_vehicle_num)
	,m_lyaw_v(g_vehicle_num)
	,m_lpos_time_v(g_vehicle_num)
	,m_lyaw_time_v(g_vehicle_num)
	,m_defaultUri_v(g_vehicle_num)
	,m_uri_v(g_vehicle_num)
	,m_output_v(g_vehicle_num)
	,m_vicon_pos_err(0.1f)
	{
		char msg_name[50];

		for(int i=0;i<g_vehicle_num;i++){
			//position estimation
			sprintf(msg_name,"/vehicle%d/pos_est",i);
			m_pos_estpub_v[i] = nh.advertise<easyfly::pos_est>(msg_name, 1);

			sprintf(msg_name,"/vicon/crazyflie%d/whole",i);
			m_viconsub_v[i] = nh.subscribe<geometry_msgs::TransformStamped>(msg_name,5,boost::bind(&SwarmLinker::viconCallback, this, _1, i));

			//attitude estimation
			sprintf(msg_name,"/vehicle%d/att_est",i);
			m_att_estpub_v[i] = nh.advertise<easyfly::att_est>(msg_name, 1);

			sprintf(msg_name,"/vehicle%d/yaw_est",i);
			m_yawsub_v[i] = nh.subscribe<crazyflie_driver::Yaw_est>(msg_name,5,boost::bind(&SwarmLinker::yaw_estCallback, this, _1, i));

			m_lpos_time_v[i] = ros::Time::now();

		}
	}

	void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg,int index)
	{
		ros::Time rightnow = ros::Time::now();
		double dt = rightnow.toSec() - m_lpos_time_v[index].toSec();
		m_lpos_time_v[index] = rightnow;
		m_pos_est_v[index].pos_est.x = msg->transform.translation.x;
		m_pos_est_v[index].pos_est.y = msg->transform.translation.y;
		m_pos_est_v[index].pos_est.z = msg->transform.translation.z;
		m_pos_est_v[index].vel_est.x = (m_pos_est_v[index].pos_est.x - m_lpos_v[index].x)/dt;
		m_pos_est_v[index].vel_est.y = (m_pos_est_v[index].pos_est.y - m_lpos_v[index].y)/dt;
		m_pos_est_v[index].vel_est.z = (m_pos_est_v[index].pos_est.z - m_lpos_v[index].z)/dt;
		m_lpos_v[index].x = m_pos_est_v[index].pos_est.x;
		m_lpos_v[index].y = m_pos_est_v[index].pos_est.y;
		m_lpos_v[index].z = m_pos_est_v[index].pos_est.z;
		//TODO
		//m_est_v[vehicle_index].yaw_est = 0;
		m_pos_estpub_v[index].publish(m_pos_est_v[index]);
	}

	void yaw_estCallback(const crazyflie_driver::Yaw_est::ConstPtr& msg,int index)
	{	
		//m_index = msg->group_index;
		ros::Time rightnow = ros::Time::now();
		double dt = rightnow.toSec() - m_lyaw_time_v[index].toSec();
		m_lyaw_time_v[index] = rightnow;
		m_att_est_v[index].att_est.z = msg->Yaw_est;
		m_att_est_v[index].yawrate_est = (m_att_est_v[index].att_est.z - m_lyaw_v[index])/dt;
		m_att_est_v[index].att_est.x = msg->Pitch_est;
		m_att_est_v[index].att_est.y = msg->Roll_est;
		m_lyaw_v[index] = m_att_est_v[index].att_est.z;

		m_att_estpub_v[index].publish(m_att_est_v[index]);
		m_lyaw_time_v[index] = ros::Time::now();
	}

	
	void vicon_est_Callback(geometry_msgs::TransformStamped::ConstPtr& msg) //get the vicon datas and calculate the vehicles.
	{
		float dt = 1.0f/m_vicon_freq;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv,"Swarm_Linker");
	ros::NodeHandle n("~");
	n.getParam("g_vehicle_num",g_vehicle_num);
	SwarmLinker swarmlinker(n);

  return 0;


}
