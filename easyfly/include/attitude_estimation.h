#include <math.h>
#include "type_methode.h"
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"
using namespace Eigen;

class Attitude_estimator 
{
public:
	Attitude_estimator()
	: m_init_North()
	, m_erroM()
	{
		const float K_Pa = 20.0f;
		const float K_Ia = 0.2f;
		const float K_Pm = 20.0f;
		const float K_Im = 0.2f;
		m_Kcoefs(0) = K_Pa;
  	 	m_Kcoefs(1) = K_Ia;
  	 	m_Kcoefs(2) = K_Pm;
  	 	m_Kcoefs(3) = K_Im; 
  	 	m_erroM.resize(3,4);
	};
	/*Vector4f m_Kcoefs;
	Vector3f m_init_North;
	MatrixXf m_erroM;*/
protected:
	Vector3f m_init_North;
	Vector4f m_Kcoefs;
	MatrixXf m_erroM;
	Vector3f err_a, err_g, upright, err_a_Int;
  	Vector3f err_m_earth, err_mag_body, mag_earth, err_m_Int;
  	Vector4f q_dot;
  	float mag_dot_k;
	void attitude_reset(const Vector3f* acc, const Vector3f* mag, Att* att)
{
  	Vector3f k, I, J;
  	k = -*acc;
  	vec3f_normalize(&k);  //direction sky in the B reference

  	mag_dot_k = vec3f_dot(mag, &k);
  	I = *mag - mag_dot_k * k; //direction north in the B reference
  	vec3f_normalize(&I);
  	m_init_North = I;
  	//printf("MAG_xy: %f  %f  %f\n", I(0), I(1), I(2) );
  	vec3f_cross(&k, &I, &J);	//J: direction west in the B reference
  	for (int i=0; i<3; i++){
  	(att->R)(0,i) = I(i);
  	(att->R)(1,i) = J(i);
  	(att->R)(2,i) = k(i);
  	}
  	rotation2quaternion(&att->R, &att->Q);
  	rotation2euler(&att->R, &att->Euler);
}

	void  attitude_estimation(Att* att, Vector3f* acc, Vector3f* gyro, Vector3f* mag, float dt)
{
	//attitude_reset(acc,mag,att);
	//dt:<0.01-0.02
  	vec3f_normalize(acc);
  	vec3f_normalize(mag);
  	err_a.setZero();
  	err_g.setZero();
  	err_m_Int.setZero();
  	err_a_Int.setZero();
  	err_m_earth.setZero();

  	upright = (att->R).row(2); //tested:  good
  	//printf("UPRIGHT:  %f   %f   %f\n",upright(0),upright(1),upright(2));
  	vec3f_cross(acc, &upright, &err_a);

  	mag_dot_k = vec3f_dot(mag,&upright);
  	*mag = *mag - mag_dot_k * upright;
  	body2earth(&att->R, mag, &mag_earth,3);
  	vec3f_cross(&m_init_North, &mag_earth, &err_m_earth);
  	//printf("ERR_MAG_EARTH:  %f   %f   %f\n",err_m_earth(0),err_m_earth(1),err_m_earth(2) );
  	err_m_Int += err_m_earth*dt;
  	err_a_Int += err_a*dt;
  	//printf("ERRInt_MAG_EARTH:  %f   %f   %f\n",err_m_Int(0),err_m_Int(1),err_m_Int(2) );
	for(int i=0; i<3; i++)
	{
  	m_erroM(i,0) = err_a(i);
  	m_erroM(i,1) = err_a_Int(i);
  	m_erroM(i,2) = err_m_earth(i);
  	m_erroM(i,3) = err_m_Int(i);
  	}
    if ((*gyro)(0) == (*gyro)(0) && (*gyro)(1) == (*gyro)(1) && (*gyro)(2) == (*gyro)(2))
    {(*gyro) = (*gyro) - m_erroM * m_Kcoefs;
    //printf("GYRO:   %f   %f   %f\n", (*gyro)(0), (*gyro)(1), (*gyro)(2));
    Matrix4f M_Q2Qdot;
    M_Q2Qdot << 0,-(*gyro)(0),-(*gyro)(1),-(*gyro)(2),
                (*gyro)(0),0,(*gyro)(2),-(*gyro)(1),
                (*gyro)(1),-(*gyro)(2),0,(*gyro)(0),
                (*gyro)(2),(*gyro)(1),-(*gyro)(0),0;
    M_Q2Qdot /= 2.0f;  //minus for the vicon
    q_dot.setZero();    
    q_dot = M_Q2Qdot*(att->Q);
    att->Q = att->Q + q_dot*dt;
    quaternion_normalize(&att->Q);
    quaternion2rotation(&(att->Q),&(att->R));
    rotation2euler(&(att->R),&(att->Euler));
    } 	
	}
};

