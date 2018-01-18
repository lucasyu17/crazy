#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <easyfly/commands.h>
#include <easyfly/state_est.h>
#include <easyfly/output.h>

#include <geometry_msgs/TransformStamped.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include "commons.h"

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyradio.h>
#include <crazyflie_cpp/Crazyflie.h>
int g_vehicle_num=2;
class Linker
{
private:
	std::vector<ros::Publisher> m_estpub_v;
	std::vector<ros::Subscriber> m_viconsub_v, m_outputsub_v;
	std::vector<easyfly::state_est> m_est_v;
	std::vector<easyfly::output> m_output_v;
	std::vector<geometry_msgs::Vector3> m_lpos_v;
	std::vector<ros::Time> m_lpos_time_v;
	std::vector<std::string> m_defaultUri_v, m_uri_v;
	std::vector<Crazyflie> m_cf_v;
public:
	Linker(ros::NodeHandle& nh)
	:m_estpub_v(g_vehicle_num)
	,m_est_v(g_vehicle_num)
	,m_viconsub_v(g_vehicle_num)
	,m_lpos_v(g_vehicle_num)
	,m_lpos_time_v(g_vehicle_num)
	,m_defaultUri_v(g_vehicle_num)
	,m_uri_v(g_vehicle_num)
	,m_output_v(g_vehicle_num)
	,m_outputsub_v(g_vehicle_num)
	{
		char msg_name[50];
		for(int i=0;i<g_vehicle_num;i++){
			sprintf(msg_name,"/vehicle%d/state_est",i);
			m_estpub_v[i] = nh.advertise<easyfly::state_est>(msg_name, 1);
			sprintf(msg_name,"/vicon/crazyflie%d/whole",i);
			m_viconsub_v[i] = nh.subscribe<geometry_msgs::TransformStamped>(msg_name,5,boost::bind(&Linker::viconCallback, this, _1, i));
			sprintf(msg_name,"/vehicle%d/output",i);
			m_outputsub_v[i] = nh.subscribe<easyfly::output>(msg_name,5,boost::bind(&Linker::outputCallback, this, _1, i));
			m_lpos_time_v[i] = ros::Time::now();

		}
	}
	int connect_get_param(int argc, char **argv)
	{
	//  std::string uri;
	//	std::string defaultUri1("radio://0/110/250K/E7E7E7E7E7");
	//	std::string defaultUri2("radio://1/120/250K/E7E7E7E7E7");//2M changed to 250K
		ros::NodeHandle n;
		char msg_name[50];
		for(int i=0;i<g_vehicle_num;i++){
			sprintf(msg_name,"/vehicle%d/controller%d/uri",i,i);
			n.getParam(msg_name, m_defaultUri_v[i]);
			//	m_defaultUri_v[i] = "radio://0/110/250K/E7E7E7E7E7";
		}
		namespace po = boost::program_options;

		for(int i=0;i<g_vehicle_num;i++){
			po::options_description desc("Allowed options");
			desc.add_options()
			  ("help", "produce help message")
			  ("uri", po::value<std::string>(&m_uri_v[i])->default_value(m_defaultUri_v[i]), "unique ressource identifier");
			try//without which the uri will not be valid
			{
				po::variables_map vm;
				po::store(po::parse_command_line(argc, argv, desc), vm);
				po::notify(vm);

				if (vm.count("help")) {
					std::cout << desc << "\n";
					return 0;
				}
			}
			catch(po::error& e)
			{
				std::cerr << e.what() << std::endl << std::endl;
				std::cerr << desc << std::endl;
				return 1;
			}
		}
	}
	void add_vehicles(void){
		for(int i=0;i<g_vehicle_num;i++){
			Crazyflie cf(m_uri_v[i]);
			m_cf_v.push_back(cf);
			m_cf_v[i].requestParamToc();
		}	
	}
	void run(double frequency)
	{
		ros::NodeHandle node;
		ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Linker::iteration, this);
		ros::spin();
	}
	void iteration(const ros::TimerEvent& e)
	{
		static float time_elapse = 0;
		float dt = e.current_real.toSec() - e.last_real.toSec();
		time_elapse += dt;
		for(int i=0;i<g_vehicle_num;i++){
			m_cf_v[i].sendSetpoint(
				m_output_v[i].att_sp.x,
				m_output_v[i].att_sp.y,
				m_output_v[i].att_sp.z,
				m_output_v[i].throttle);
		}
	}
	void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg, int vehicle_index)
	{
		ros::Time rightnow = ros::Time::now();
		double dt = rightnow.toSec() - m_lpos_time_v[vehicle_index].toSec();
		m_lpos_time_v[vehicle_index] = rightnow;
		m_est_v[vehicle_index].pos_est.x = msg->transform.translation.x;
		m_est_v[vehicle_index].pos_est.y = msg->transform.translation.y;
		m_est_v[vehicle_index].pos_est.z = msg->transform.translation.z;
		m_est_v[vehicle_index].vel_est.x = (m_est_v[vehicle_index].pos_est.x - m_lpos_v[vehicle_index].x)/dt;
		m_est_v[vehicle_index].vel_est.y = (m_est_v[vehicle_index].pos_est.y - m_lpos_v[vehicle_index].y)/dt;
		m_est_v[vehicle_index].vel_est.z = (m_est_v[vehicle_index].pos_est.z - m_lpos_v[vehicle_index].z)/dt;
		m_lpos_v[vehicle_index].x = m_est_v[vehicle_index].pos_est.x;
		m_lpos_v[vehicle_index].y = m_est_v[vehicle_index].pos_est.y;
		m_lpos_v[vehicle_index].z = m_est_v[vehicle_index].pos_est.z;
		
		//TODO
		//m_est_v[vehicle_index].yaw_est = 0;

		m_estpub_v[vehicle_index].publish(m_est_v[vehicle_index]);
	}
	void outputCallback(const easyfly::output::ConstPtr& msg, int vehicle_index)
	{
		m_output_v[vehicle_index].att_sp.x = msg->att_sp.x;
		m_output_v[vehicle_index].att_sp.y = msg->att_sp.y;
		m_output_v[vehicle_index].att_sp.z = msg->att_sp.z;
		m_output_v[vehicle_index].throttle = msg->throttle;
	}
};


int main(int argc, char **argv)
{
//  int ret = init_scan(argc, argv);
	ros::init(argc, argv, "linker");
	ros::NodeHandle n("~");
	n.getParam("/vehicle_num", g_vehicle_num);
	Linker linker(n);
	int ret = linker.connect_get_param(argc, argv);
	linker.add_vehicles();
	
//	n.getParam("/flight_mode", g_flight_mode);this has moved to function run
	
	linker.run(50);


  return 0;


}
