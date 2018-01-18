#ifndef BASIC_TYPES_H
#define BASIC_TYPES_H
#pragma anon_unions
#include <stdint.h>
#include <stdbool.h>
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"
using namespace Eigen;

struct Att {
    Matrix3f R;
    Vector4f Q;
    Vector3f Euler;
  } ;


struct M_est_Vecs{
		Vector3f m_pos_est, m_cfImuAcc, m_att_est;//, m_vel_est, m_att_est, m_lpos, m_latt;//, m_cfImuAcc_wd, m_cfImuVel, m_cfImuVel_wd, m_cfImuPos, m_cfImuPos_wd;
		Matrix3f m_R_est;
		//rotation_t R_est;
};

struct M_sp_Vecs{
		Vector4f v_posctrl_posSp;
		Vector3f v_posctrl_velFF;
		Vector3f v_posctrl_acc_sp;
		//Vector3f v_vel_sp, v_acc_sp, v_att_sp, v_posctrl_velFF;
		//Vector3f v_lposSp, v_lvelSp;
	};
struct M_recording
	{
		Vector3f Rec_posEst;
		Vector3f Rec_velEst;
		Vector3f Rec_attEst;
		Vector3f Rec_posSp;
		Vector3f Rec_velSp;
		Vector3f Rec_accSp;
		float Rec_yaw_sp;
		float Rec_roll_sp;
		float Rec_pitch_sp;
	};
	

#endif
