#pragma once

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>

#include "FileIO/fileio.h"
#include "Numerical/numerical.h"

using namespace std;

class RobotArm
{
public:
    RobotArm(uint numbody, uint DOF);
    ~RobotArm();
#ifdef FILEIO_H_
    void run_kinematics();
    void run_inverse_kinematics();
    void run_dynamics();
    void run_inverse_kinematics_with_path_generator();
    void run_virtual_spring_damper_algorithm();
    void run_feeding_motion();
#endif
    void run_kinematics(double *q, double *des_pose);
    void run_inverse_kinematics(double* cur_joint, double* des_pose, double* res_joint, double* res_pose);
    void path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path);

private:
    inline void tilde(double *a, double *b) {
        b[0] = 0;       b[1] = -a[2];	b[2] = a[1];
        b[3] = a[2];	b[4] = 0;       b[5] = -a[0];
        b[6] = -a[1];	b[7] = a[0];	b[8] = 0;
    }

    class Body
    {
    public:
        Body();
        Body(double psi, double theta, double phi, double sijp_x, double sijp_y, double sijp_z);
        ~Body();

        double qi, qi_dot, qi_ddot;
        double mi, Ixx, Iyy, Izz, Ixy, Iyz, Izx;

        // orientation
        double Ai[9], Aijpp[9], Ai_Cij[9], Cij[9], u_vec[3];
        // position
        double sij[3], sijp[3], ri[3], re[3], Ae[9], re_dot[3], we[3], ori[3], pose[6];
        // jacobian
        double Jvi[3], Jwi[3], re_qi[3], Ae_qi[9], r6_qi[3], A6_qi[9], Ae_qi_Cij[9], Ae_qi_Cij_Aijpp[9], Aijpp_qi[9], Cij_Aijpp[9], Ai_Cij_Aijpp_qi[9], Ae_qi_sijp[3], Ae_qi_end[3];
        double Ae_qi_31, Ae_qi_32, Ae_qi_33, Ae_qi_21, Ae_qi_11, roll_qi_1, roll_qi_2, roll_qi, pitch_qi_1, pitch_qi_2, pitch_qi, yaw_qi_1, yaw_qi_2, yaw_qi;
        double oi[3], zi[3], zit[9];
        // velocity state
        double Hi[3], rit[9], Bi[6], Yih[6];
        // cartesian velocity
        double Ti[36], Yib[6], ri_dot[3], wi[3], wit[9], rhoip[3], rhoi[3], ric[3], ric_dot[3];
        // mass & force
        double Cii[9], Ai_Cii[9], Jic[9], Jip[9], rit_dot[9], rict_dot[9], rict[9], Mih[36], fic[3], tic[3], Qih[6], Qih_g[6], Qih_c[6];
        // velocity coupling
        double Hi_dot[3], Di[6], Di_sum[6];
        // system EQM
        double Ki[36], Li[6], Li_g[6], Li_c[6], Ki_Di[6], Ki_Di_sum[6];

        static void ang2mat(double ang_z1, double ang_x, double ang_z2, double* mat, bool deg_flag = true);
    };

    double DH[6*4];

    uint num_body, dof;
    double *PH, *PH_pos, *PH_ori, *delta_q, *J, *JD;
    double *M, *Q, *Q_c, *Q_g;

    // system variable
    double start_time, end_time, h, t_current;
    double g;

    // file
//    char file_name[256];
    std::string file_name;
    FILE *fp, *fp1, *fp2, *fp3, *fp4, *fp5, *fp6;

    Body *body;
    Numerical *numeric;

    double lamda;

    void kinematics();
    void inverse_kinematics(double pos_d[3], double ori_d[3]);
    void inverse_kinematics(double desired[6]);
        void convert_desired(double desired_origin[6], double rot_mat[9], double desired_conv[6]);
        void jacobian();
    void dynamics();
    void dynamics(double *Ta);
    void save_data();

    void mat(double *mat_1, double *mat_2, uint row_1, uint col_1, uint row_2, uint col_2, double *mat_3);
    void mat(double *mat_1, double *vec_2, uint row_1, uint col_1, uint row_2, double *vec_3);
    void rpy2mat(double yaw, double pitch, double roll, double *mat);
    void mat_to_axis_angle(double R_init[9], double R_final[9], double r[3], double *theta);
    void axis_angle_to_mat(double r[3], double theta, double mat[9]);
    void mat2rpy(double mat[9], double ori[3]);

    double Ae_31, Ae_32, Ae_33, Ae_21, Ae_11, Ae_32_33, Ae_32_33_2, Ae_21_11;
    double roll_q_temp1, roll_q_temp2, roll_q_temp3, roll_q_temp4;
    double pitch_q_temp1, pitch_q_temp2, pitch_q_temp3, pitch_q_temp4;
    double yaw_q_temp1, yaw_q_temp2, yaw_q_temp3, yaw_q_temp4;

    typedef struct _StructPathGenerateData{
        std::vector<double> path_x, path_y, path_z, path_roll, path_pitch, path_yaw, path_theta;
        double r[3], R_init[9];
        unsigned int data_size;
    }StructPathGenerateData;

    vector<StructPathGenerateData> movePath;
};

