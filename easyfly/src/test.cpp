#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>	
#include <easyfly/pos_est.h>	

int g_vehicle_num = 1;
class Test
{
public:
	Test():
	 m_estsub_v(g_vehicle_num)
	,m_est_v(g_vehicle_num)
	{
	ros::NodeHandle nh;
	char msg_name[50];

	for(int i=0;i<g_vehicle_num;i++){
			sprintf(msg_name,"/vehicle%d/pos_est",i);
			m_estsub_v[i] = nh.subscribe<easyfly::pos_est>(msg_name,5,boost::bind(&Test::pos_estCallback, this, _1, i));
			printf("Done\n");
		}
		ros::spin();
	}
public:
	std::vector<ros::Subscriber> m_estsub_v; 
	std::vector<easyfly::pos_est> m_est_v;
	void pos_estCallback(const easyfly::pos_est::ConstPtr& msg, int vehicle_index)
	{
		easyfly::pos_est pos_estmsg;
		printf("POS_EST-X:  %f\n", msg->pos_est.x);
		printf("POS_EST-Y:  %f\n", msg->pos_est.y);
		printf("POS_EST-Z:  %f\n", msg->pos_est.z);
		printf("VEL_EST-X:  %f\n", msg->vel_est.x);
		printf("VEL_EST-Y:  %f\n", msg->vel_est.y);
		printf("VEL_EST-Z:  %f\n", msg->vel_est.z);
		m_est_v[vehicle_index].pos_est.x = msg->pos_est.x;
		m_est_v[vehicle_index].pos_est.y = msg->pos_est.y;
		m_est_v[vehicle_index].pos_est.z = msg->pos_est.z;
		m_est_v[vehicle_index].vel_est.x = msg->vel_est.x;
		m_est_v[vehicle_index].vel_est.y = msg->vel_est.y;
		m_est_v[vehicle_index].vel_est.z = msg->vel_est.z;
	}
};
int main(int argc, char **argv)
{
	/* code */
	ros::init(argc, argv, "test");
	Test test;
	return 0;
}