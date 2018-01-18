#include "ros/ros.h"

using namespace std;

class PID
{
private:
	float m_kp;
	float m_kd;
	float m_ki;
	float m_kpp;
	float m_ff;
	float m_minOutput;
	float m_maxOutput;
	float m_integratorMin;
	float m_integratorMax;
	float m_integral;
	float m_previousError;
	ros::Time m_previousTime;
	friend class Swarm_Controller;

public:
	PID(
		float kp,
		float kd,
		float ki,
		float kpp,
		float ff,
		float minOutput,
		float maxOutput,
		float integratorMin,
		float integratorMax)
		: m_kp(kp)
		, m_kd(kd)
		, m_ki(ki)
		, m_kpp(kpp)
		, m_ff(ff)
		, m_minOutput(minOutput)
		, m_maxOutput(maxOutput)
		, m_integratorMin(integratorMin)
		, m_integratorMax(integratorMax)
		, m_integral(0)
		, m_previousError(0)
		, m_previousTime(ros::Time::now())
	{
	}

	void reset()
	{
		m_integral = 0;
		m_previousError = 0;
		m_previousTime = ros::Time::now();
	}

	void setIntegral(float integral)
	{
		m_integral = integral;
	}

	float ki() const
	{
		return m_ki;
	}
	float ff() const
	{
		return m_ff;
	}
	float pid_update(float est, float setpt, float dt)
	{
		/*ros::Time time = ros::Time::now();
		float dt = time.toSec() - m_previousTime.toSec();*/
		
			float error = setpt - est;
			m_integral += error * dt;
			m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
			float p = m_kp * error;
			float d = 0;
			if (dt > 0){
				d = m_kd * (error - m_previousError) / dt;
			}
			float i = m_ki * m_integral;
			float output = p + d + i;
			m_previousError = error;
			//m_previousTime = time;
			float result = std::max(std::min(output, m_maxOutput), m_minOutput);
			return result;	
		
	}
	float pp_update(float est, float setpt)
	{
		float error = setpt - est;
		float output = m_kpp * error;
		
		return output;
	}
	
	};