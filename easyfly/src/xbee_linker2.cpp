#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <termios.h>
#include <boost/program_options.hpp>


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>


int port_desc0, port_desc1;
int g_vehicle_num=2;
int mySerialOpen (const char *device, const int baud)
{
  struct termios options ;
  speed_t myBaud ;
  int     status, fd ;

  switch (baud)
  {
    case     50:	myBaud =     B50 ; break ;
    case     75:	myBaud =     B75 ; break ;
    case    110:	myBaud =    B110 ; break ;
    case    134:	myBaud =    B134 ; break ;
    case    150:	myBaud =    B150 ; break ;
    case    200:	myBaud =    B200 ; break ;
    case    300:	myBaud =    B300 ; break ;
    case    600:	myBaud =    B600 ; break ;
    case   1200:	myBaud =   B1200 ; break ;
    case   1800:	myBaud =   B1800 ; break ;
    case   2400:	myBaud =   B2400 ; break ;
    case   4800:	myBaud =   B4800 ; break ;
    case   9600:	myBaud =   B9600 ; break ;
    case  19200:	myBaud =  B19200 ; break ;
    case  38400:	myBaud =  B38400 ; break ;
    case  57600:	myBaud =  B57600 ; break ;
    case 115200:	myBaud = B115200 ; break ;
    case 230400:	myBaud = B230400 ; break ;

    default:
      return -2 ;
  }
  printf("now open\n");
  if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    return -1 ;
  printf("now fcntl\n");
  fcntl (fd, F_SETFL, O_RDWR) ;

// Get and modify current options:
  printf("now tcgetattr\n");
  tcgetattr (fd, &options) ;
printf("now cfmakeraw\n");
    cfmakeraw   (&options) ;
    printf("now cfsetispeed\n");
    cfsetispeed (&options, myBaud) ;
    printf("now cfsetospeed\n");
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)
printf("now tcsetattr\n");
  tcsetattr(fd, TCSANOW,&options);
//  tcsetattr (fd, TCSANOW | TCSAFLUSH, &options) ;
printf("now ioctl\n");
  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;
printf("now fcntl again\n");
  ioctl (fd, TIOCMSET, &status);
printf("now finished\n");
  usleep (10000) ;	// 10mS

  return fd ;
}
class Linker
{
private:
	/*
	std::vector<ros::Publisher> m_estpub_v;
	std::vector<ros::Subscriber> m_viconsub_v, m_outputsub_v;
	std::vector<easyfly::state_est> m_est_v;
	std::vector<easyfly::output> m_output_v;
	std::vector<geometry_msgs::Vector3> m_lpos_v;
	std::vector<ros::Time> m_lpos_time_v;
	std::vector<std::string> m_defaultUri_v, m_uri_v;
	std::vector<Crazyflie> m_cf_v;
	*/
public:
	Linker(ros::NodeHandle& nh)
/*	:m_estpub_v(g_vehicle_num)
	,m_est_v(g_vehicle_num)
	,m_viconsub_v(g_vehicle_num)
	,m_lpos_v(g_vehicle_num)
	,m_lpos_time_v(g_vehicle_num)
	,m_defaultUri_v(g_vehicle_num)
	,m_uri_v(g_vehicle_num)
	,m_output_v(g_vehicle_num)
	,m_outputsub_v(g_vehicle_num)*/
	{
/*		char msg_name[50];
		for(int i=0;i<g_vehicle_num;i++){
			sprintf(msg_name,"/vehicle%d/state_est",i);
			m_estpub_v[i] = nh.advertise<easyfly::state_est>(msg_name, 1);
			sprintf(msg_name,"/vicon/crazyflie%d/whole",i);
			m_viconsub_v[i] = nh.subscribe<geometry_msgs::TransformStamped>(msg_name,5,boost::bind(&Linker::viconCallback, this, _1, i));
			sprintf(msg_name,"/vehicle%d/output",i);
			m_outputsub_v[i] = nh.subscribe<easyfly::output>(msg_name,5,boost::bind(&Linker::outputCallback, this, _1, i));
			m_lpos_time_v[i] = ros::Time::now();

		}
		*/
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
		unsigned char out_buf[16];
		unsigned char in_buf[16];
		for(int i=0; i<16; i++) {
			out_buf[i] = i;
		}
		write(port_desc0, out_buf, 16);
	//	read(port_desc0, in_buf, 16);
	//	for(int i=0; i<16; i++) {
	//		printf("%d\n", in_buf[i]);
	//	}
		
		
/*		for(int i=0;i<g_vehicle_num;i++){
			m_cf_v[i].sendSetpoint(
				m_output_v[i].att_sp.x * RAD2DEG,
				m_output_v[i].att_sp.y * RAD2DEG,
				m_output_v[i].att_sp.z * RAD2DEG,
				m_output_v[i].throttle * 40000);
		}*/
	}

};


int main(int argc, char **argv)
{
//  int ret = init_scan(argc, argv);
	
	ros::init(argc, argv, "xbee_linker1");
	ros::NodeHandle n("~");
	if ((port_desc0 = mySerialOpen ("/dev/ttyUSB1", 57600)) >= 0){
		printf ("USB1 opened");
	}
/*	if ((port_desc1 = mySerialOpen ("/dev/ttyUSB1", 57600)) >= 0){
		printf ("USB1 opened");
	}*/
//	n.getParam("/vehicle_num", g_vehicle_num);
	Linker linker(n);
//	int ret = linker.connect_get_param(argc, argv);
//	linker.add_vehicles();
	
//	n.getParam("/flight_mode", g_flight_mode);this has moved to function run
	
	linker.run(2);


  return 0;


}




