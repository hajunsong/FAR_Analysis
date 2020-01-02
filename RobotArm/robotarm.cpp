#include "robotarm.h"

RobotArm::Body::Body(){
    u_vec[0] = 0;
    u_vec[1] = 0;
    u_vec[2] = 1;
}

RobotArm::Body::~Body(){

}

void RobotArm::Body::ang2mat(double ang_z1, double ang_x, double ang_z2, double *mat, bool deg_flag)
{
    double z1, x, z2;
    if (deg_flag){
        z1 = ang_z1*M_PI/180.0;
        x = ang_x*M_PI/180.0;
        z2 = ang_z2*M_PI/180.0;
    }
    else{
        z1 = ang_z1;
        x = ang_x;
        z2 = ang_z2;
    }

    double Rz1[9] = {cos(z1), -sin(z1), 0, sin(z1), cos(z1), 0, 0, 0, 1};
    double Rx[9] = {1, 0, 0, 0, cos(x), -sin(x), 0, sin(x), cos(x)};
    double Rz2[9] = {cos(z2), -sin(z2), 0, sin(z2), cos(z2), 0, 0, 0, 1};
    double Rz1Rx[9] = {0,};
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                Rz1Rx[i*3+j] += Rz1[i*3+k]*Rx[k*3+j];
            }
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            mat[i*3+j] = 0;
            for(int k = 0; k < 3; k++){
                mat[i*3+j] += Rz1Rx[i*3+k]*Rz2[k*3+j];
            }
        }
    }
}

void RobotArm::mat(double *mat_1, double *mat_2, uint row_1, uint col_1, uint row_2, uint col_2, double *mat_3){
    if (col_1 != row_2){
        printf("please check matrix size\n");
        return;
    }

    double temp;
    for(uint i = 0; i < row_1; i++){
        for(uint j = 0; j < col_2; j++){
            temp = 0;
            for(uint k = 0; k < row_2; k++){
                temp += mat_1[i*col_1 + k]*mat_2[k*col_2 + j];
            }
            mat_3[i*col_2 + j] = temp;
        }
    }
}

void RobotArm::mat(double *mat_1, double *vec_2, uint row_1, uint col_1, uint row_2, double *vec_3){
    if (col_1 != row_2){
        printf("please check matrix size\n");
        return;
    }

    double temp;
    for(uint i = 0; i < row_1; i++){
        temp = 0;
        for(uint j = 0; j < row_2; j++){
            temp += mat_1[i*col_1 + j]*vec_2[j];
        }
        vec_3[i] = temp;
    }
}

void RobotArm::rpy2mat(double yaw, double pitch, double roll, double *mat)
{
    double R_yaw[9] = {cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1};
    double R_pitch[9] = {cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch)};
    double R_roll[9] = {1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll)};
    double R_yaw_R_pitch[9] = {0,};
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                R_yaw_R_pitch[i*3+j] += R_yaw[i*3+k]*R_pitch[k*3+j];
            }
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            mat[i*3+j] = 0;
            for(int k = 0; k < 3; k++){
                mat[i*3+j] += R_yaw_R_pitch[i*3+k]*R_roll[k*3+j];
            }
        }
    }
}

void RobotArm::mat_to_axis_angle(double R_init[], double R_final[], double r[], double *theta)
{
    double R[9] = {0,};
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                R[i*3+j] += R_init[k*3+i]*R_final[k*3+j];
            }
        }
    }

    double m00, m01, m02, m10, m11, m12, m20, m21, m22;

    m00 = R[0*3 + 0]; m01 = R[0*3 + 1]; m02 = R[0*3 + 2];
    m10 = R[1*3 + 0]; m11 = R[1*3 + 1]; m12 = R[1*3 + 2];
    m20 = R[2*3 + 0]; m21 = R[2*3 + 1]; m22 = R[2*3 + 2];

    *theta = acos((m00 + m11 + m22 - 1)/2);

    r[0] = (m21 - m12)/sqrt(pow((m21 - m12), 2)+pow((m02 - m20), 2)+pow((m10 - m01), 2));
    r[1] = (m02 - m20)/sqrt(pow((m21 - m12), 2)+pow((m02 - m20), 2)+pow((m10 - m01), 2));
    r[2] = (m10 - m01)/sqrt(pow((m21 - m12), 2)+pow((m02 - m20), 2)+pow((m10 - m01), 2));
}

void RobotArm::axis_angle_to_mat(double r[], double theta, double mat[])
{
    double c = cos(theta);
    double s = sin(theta);
    double t = 1 - c;
    double mag = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
    double x = r[0]/mag;
    double y = r[1]/mag;
    double z = r[2]/mag;

    mat[0] = t*x*x + c;
    mat[1] = t*x*y - z*s;
    mat[2] = t*x*z + y*s;

    mat[3] = t*x*y + z*s;
    mat[4] = t*y*y + c;
    mat[5] = t*y*z - x*s;

    mat[6] = t*x*z - y*s;
    mat[7] = t*y*z + x*s;
    mat[8] = t*z*z + c;
}

void RobotArm::mat2rpy(double mat[], double ori[])
{
    ori[0] = atan2(mat[2 * 3 + 1], mat[2 * 3 + 2]);
    ori[1] = atan2(-mat[2 * 3 + 0], sqrt(pow(mat[2 * 3 + 1], 2.0) + pow(mat[2 * 3 + 2], 2.0)));
    ori[2] = atan2(mat[1 * 3 + 0], mat[0 * 3 + 0]);
}

RobotArm::RobotArm(uint numbody, uint DOF) {
    num_body = numbody;
    dof = DOF;

    PH = new double[dof];
    PH_pos = new double[3 * num_body];
    PH_ori = new double[3 * num_body];
    delta_q = new double[dof];
    J = new double[num_body * dof];
    JD = new double[dof * num_body];

    M = new double[num_body*num_body];
    Q = new double[num_body];
    Q_c = new double[num_body];
    Q_g = new double[num_body];

    body = new Body[num_body+1];

    lamda = 0.0001;

    // read data
    start_time = 0;
    h = 0.005;
    g = -9.80665;

    // DH paramter
    // | Link | alpha(deg) |  a(mm)  |  d(mm)   | theta(deg) |
    // |=====================================================|
    // |  1   |    -90     |  0      |   0      |     90     |
    // |  2   |     0      |  151.75 |   0      |    -90     |
    // |  3   |     0      |  150    |   0      |     0      |
    // |  4   |     90     |  86.75  |   0      |     90     |
    // |  5   |     90     |  0      |   -20.25 |     90     |
    // |  6   |     0      |  0      |   102.5  |     0      |

    DH[0] = -90;    DH[1] = 0;          DH[2] = 0;          DH[3] = 90;
    DH[4] = 0;      DH[5] = 0.15175;    DH[6] = 0;          DH[7] = -90;
    DH[8] = 0;      DH[9] = 0.150;      DH[10] = 0;         DH[11] = 0;
    DH[12] = 90;    DH[13] = 0.08675;   DH[14] = 0;         DH[15] = 90;
    DH[16] = 90;    DH[17] = 0;         DH[18] = -0.02025;  DH[19] = 90;
    DH[20] = 0;     DH[21] = 0;         DH[22] = 0.1025;    DH[23] = 0;

    // body 0 variable
    body[0].Ai[0] = 1; body[0].Ai[1] = 0; body[0].Ai[2] = 0;
    body[0].Ai[3] = 0; body[0].Ai[4] = 1; body[0].Ai[5] = 0;
    body[0].Ai[6] = 0; body[0].Ai[7] = 0; body[0].Ai[8] = 1;

    body[0].ri[0] = 0; body[0].ri[1] = 0; body[0].ri[2] = 0;

    // preliminary work
    memset(body[0].Yih, 0, sizeof(double)*6);
    memset(body[0].wi, 0, sizeof(double)*3);
    memset(body[0].wit, 0, sizeof(double)*9);

    Body::ang2mat(0, 0, 0, body[0].Cij);
    body[0].sijp[0] = 0; body[0].sijp[1] = 0; body[0].sijp[2] = 0;

    body[0].ri_dot[0] = 0; body[0].ri_dot[1] = 0; body[0].ri_dot[2] = 0;
    body[0].wi[0] = 0; body[0].wi[1] = 0; body[0].wi[2] = 0;

    body[0].u_vec[0] = 0; body[0].u_vec[1] = 0; body[0].u_vec[2] = 0;

    // body 1 variables
    Body::ang2mat(DH[0*4+3], DH[0*4+0], 0, body[1].Cij);
    body[1].sijp[0] = 0; body[1].sijp[1] = 0; body[1].sijp[2] = 0;

    Body::ang2mat(-M_PI_2, M_PI_2, 0, body[1].Cii, false);
    body[1].rhoip[0] = -0.00189888; body[1].rhoip[1] = -1.44683e-8; body[1].rhoip[2] = -0.0234351;
    body[1].mi = 6.33612131907843e-002;
    body[1].Ixx = 5.46760988093101e-005;
    body[1].Iyy = 4.11897872591055e-005;
    body[1].Izz = 2.28294446378339e-005;
    body[1].Ixy = 1.16933891602143e-011;
    body[1].Iyz = 1.72355337398552e-006;
    body[1].Izx = 3.03099214889948e-011;
    body[1].Jip[0] = body[1].Ixx; body[1].Jip[1] = body[1].Ixy; body[1].Jip[2] = body[1].Izx;
    body[1].Jip[3] = body[1].Ixy; body[1].Jip[4] = body[1].Iyy; body[1].Jip[5] = body[1].Iyz;
    body[1].Jip[6] = body[1].Izx; body[1].Jip[7] = body[1].Iyz; body[1].Jip[8] = body[1].Izz;
    body[1].u_vec[0] = 0; body[1].u_vec[1] = 0; body[1].u_vec[2] = 1;


    // body 2 variables
    Body::ang2mat(DH[1*4+3], DH[1*4+0], 0, body[2].Cij);
    body[2].sijp[0] = 0; body[2].sijp[1] = -DH[1*4+1]; body[2].sijp[2] = 0;

    Body::ang2mat(M_PI, 0, 0, body[2].Cii, false);
    body[2].rhoip[0] = -0.000462227; body[2].rhoip[1] = -0.0427355; body[2].rhoip[2] = 0.000759913;
    body[2].mi = 0.291144481135948;
    body[2].Ixx = 6.84357146533933e-004;
    body[2].Iyy = 1.19767708650701e-004;
    body[2].Izz = 6.2201207394514e-004;
    body[2].Ixy = -7.26891485430593e-006;
    body[2].Iyz = 5.05996626479478e-006;
    body[2].Izx = 1.80750423403909e-007;
    body[2].Jip[0] = body[2].Ixx; body[2].Jip[1] = body[2].Ixy; body[2].Jip[2] = body[2].Izx;
    body[2].Jip[3] = body[2].Ixy; body[2].Jip[4] = body[2].Iyy; body[2].Jip[5] = body[2].Iyz;
    body[2].Jip[6] = body[2].Izx; body[2].Jip[7] = body[2].Iyz; body[2].Jip[8] = body[2].Izz;
    body[2].u_vec[0] = 0; body[2].u_vec[1] = 0; body[2].u_vec[2] = 1;

    // body 3 variables
    Body::ang2mat(DH[2*4+3], DH[2*4+0], 0, body[3].Cij);
    body[3].sijp[0] = DH[2*4+1]; body[3].sijp[1] = 0; body[3].sijp[2] = 0;

    Body::ang2mat(-M_PI_2, 0, 0, body[3].Cii, false);
    body[3].rhoip[0] = 0.075; body[3].rhoip[1] = 0; body[3].rhoip[2] = 0.000807364;
    body[3].mi = 0.416638668104345;
    body[3].Ixx = 1.45776042402133e-003;
    body[3].Iyy = 1.15949266176089e-004;
    body[3].Izz = 1.44207442743259e-003;
    body[3].Ixy = -2.14630188922107e-014;
    body[3].Iyz = -4.86620428197596e-019;
    body[3].Izx = -5.85663447574856e-020;
    body[3].Jip[0] = body[3].Ixx; body[3].Jip[1] = body[3].Ixy; body[3].Jip[2] = body[3].Izx;
    body[3].Jip[3] = body[3].Ixy; body[3].Jip[4] = body[3].Iyy; body[3].Jip[5] = body[3].Iyz;
    body[3].Jip[6] = body[3].Izx; body[3].Jip[7] = body[3].Iyz; body[3].Jip[8] = body[3].Izz;
    body[3].u_vec[0] = 0; body[3].u_vec[1] = 0; body[3].u_vec[2] = 1;

    // body 4 variables
    Body::ang2mat(DH[3*4+3], DH[3*4+0], 0, body[4].Cij);
    body[4].sijp[0] = 0; body[4].sijp[1] = DH[3*4+1]; body[4].sijp[2] = 0;

    Body::ang2mat(-M_PI_2, 0, 0, body[4].Cii, false);
    body[4].rhoip[0] = 0.000749752; body[4].rhoip[1] = 0.0609445; body[4].rhoip[2] = 0.000415268;
    body[4].mi = 0.228993914748238;
    body[4].Ixx = 7.74704754240776e-005;
    body[4].Iyy = 2.01161940464821e-004;
    body[4].Izz = 1.88922599914013e-004;
    body[4].Ixy = 2.91568180110741e-006;
    body[4].Iyz = 7.12741866241557e-008;
    body[4].Izx = -2.20466982640091e-006;
    body[4].Jip[0] = body[4].Ixx; body[4].Jip[1] = body[4].Ixy; body[4].Jip[2] = body[4].Izx;
    body[4].Jip[3] = body[4].Ixy; body[4].Jip[4] = body[4].Iyy; body[4].Jip[5] = body[4].Iyz;
    body[4].Jip[6] = body[4].Izx; body[4].Jip[7] = body[4].Iyz; body[4].Jip[8] = body[4].Izz;
    body[4].u_vec[0] = 0; body[4].u_vec[1] = 0; body[4].u_vec[2] = 1;

    // body 5 variables
    Body::ang2mat(DH[4*4+3], DH[4*4+0], 0, body[5].Cij);
    body[5].sijp[0] = 0; body[5].sijp[1] = 0; body[5].sijp[2] = DH[4*4+2];

    Body::ang2mat(M_PI, M_PI_2, 0, body[5].Cii, false);
    body[5].rhoip[0] = 0.0555687; body[5].rhoip[1] = 0; body[5].rhoip[2] = -0.000237633;
    body[5].mi = 0.204137411295743;
    body[5].Ixx = 9.4696526893192e-005;
    body[5].Iyy = 7.92107777080459e-005;
    body[5].Izz = 1.38821213983018e-004;
    body[5].Ixy = -2.02238967554624e-005;
    body[5].Iyz = -1.20999283701959e-015;
    body[5].Izx = 1.87131808263915e-015;
    body[5].Jip[0] = body[5].Ixx; body[5].Jip[1] = body[5].Ixy; body[5].Jip[2] = body[5].Izx;
    body[5].Jip[3] = body[5].Ixy; body[5].Jip[4] = body[5].Iyy; body[5].Jip[5] = body[5].Iyz;
    body[5].Jip[6] = body[5].Izx; body[5].Jip[7] = body[5].Iyz; body[5].Jip[8] = body[5].Izz;
    body[5].u_vec[0] = 0; body[5].u_vec[1] = 0; body[5].u_vec[2] = 1;

    // body 6 variables
    Body::ang2mat(DH[5*4+3], DH[5*4+0], 0, body[6].Cij);
    body[6].sijp[0] = 0; body[6].sijp[1] = 0; body[6].sijp[2] = DH[5*4+2];

    Body::ang2mat(M_PI_2, M_PI_2, -M_PI_2, body[6].Cii, false);
    body[6].rhoip[0] = 5.39394e-10; body[6].rhoip[1] = 3.33671e-8; body[6].rhoip[2] = 0.089384;
    body[6].mi = 2.08363885223627e-002;
    body[6].Ixx = 2.66302463617021e-006;
    body[6].Iyy = 1.56637607668211e-006;
    body[6].Izz = 1.88187616526518e-006;
    body[6].Ixy = 2.4095425326714e-012;
    body[6].Iyz = 2.738802635816e-013;
    body[6].Izx = 9.27461478843821e-014;
    body[6].Jip[0] = body[6].Ixx; body[6].Jip[1] = body[6].Ixy; body[6].Jip[2] = body[6].Izx;
    body[6].Jip[3] = body[6].Ixy; body[6].Jip[4] = body[6].Iyy; body[6].Jip[5] = body[6].Iyz;
    body[6].Jip[6] = body[6].Izx; body[6].Jip[7] = body[6].Iyz; body[6].Jip[8] = body[6].Izz;
    body[6].u_vec[0] = 0; body[6].u_vec[1] = 0; body[6].u_vec[2] = 1;

    numeric = new Numerical();
    numeric->absh3Initialize(h, num_body);
}

RobotArm::~RobotArm() {
    delete[] PH;
    delete[] PH_pos;
    delete[] PH_ori;
    delete[] delta_q;
    delete[] J;
    delete[] JD;

    delete[] M;
    delete[] Q;
    delete[] Q_c;
    delete[] Q_g;

    delete[] body;
    delete numeric;
}

#ifdef FILEIO_H_
void RobotArm::run_kinematics()
{
    file_name = "../FAR_Analysis/data/evaluation_motion_cpp.txt";
    fp = fopen(file_name.c_str(), "w+");

    vector<double> ref_data;
    uint row = 2501;
    uint col = 14;
    load_data("../FAR_Analysis/data/evaluation_motion_recurdyn.txt", &ref_data, "\t");

    for(uint indx = 0; indx < row; indx++) {
        for (uint i = 1; i <= 6; i++) {
            body[i].qi = ref_data[indx*col + i + 1];
        }

        kinematics();

        save_data();

        printf("Time : %.3f[s]\n", static_cast<double>(t_current));

        t_current += h;
    }

    ref_data.clear();
    fclose(fp);
}
#endif

void RobotArm::run_kinematics(double *q, double *des_pose){
    for(int i = 0; i < 6; i++){
        body[i+1].qi = q[i];
    }

    kinematics();

    des_pose[0] = body[6].re[0];
    des_pose[1] = body[6].re[1];
    des_pose[2] = body[6].re[2];
    des_pose[3] = body[6].ori[0];
    des_pose[4] = body[6].ori[1];
    des_pose[5] = body[6].ori[2];
}

#ifdef FILEIO_H_
void RobotArm::run_inverse_kinematics() {
    file_name = "../FAR_Analysis/data/evaluation_motion_cpp.txt";
    fp = fopen(file_name.c_str(), "w+");

    vector<double> ref_data;
    uint row = 2501;
    uint col = 14;
    load_data("../FAR_Analysis/data/evaluation_motion_recurdyn.txt", &ref_data, "\t");
    for (uint i = 1; i <= 6; i++) {
        body[i].qi = ref_data[0*col + i + 1];
    }

    double pos_d[3], ori_d[3];
    for (uint indx = 0; indx < row; indx++) {
        pos_d[0] = ref_data[indx*col + 8];
        pos_d[1] = ref_data[indx*col + 9];
        pos_d[2] = ref_data[indx*col + 10];
        ori_d[0] = ref_data[indx*col + 11];
        ori_d[1] = ref_data[indx*col + 12];
        ori_d[2] = ref_data[indx*col + 13];

        kinematics();

        inverse_kinematics(pos_d, ori_d);

        save_data();

        printf("Time : %.3f[s]\n", static_cast<double>(t_current));

        t_current += h;
    }

    ref_data.clear();
    fclose(fp);
}
#endif

void RobotArm::run_inverse_kinematics(double* input_q, double* des_pose, double* cur_joint, double* cur_pose){
    bool goal_reach;

    double epsilon_pos = 0.05;
    double epsilon_ang = 1;

    for (uint i = 1; i <= num_body; i++) {
        body[i].qi = input_q[i - 1];
    }

    double pos_d[3], ori_d[3];

    for(uint i = 0; i < 5; i++){
        pos_d[0] = des_pose[0];
        pos_d[1] = des_pose[1];
        pos_d[2] = des_pose[2];
        ori_d[0] = des_pose[3];
        ori_d[1] = des_pose[4];
        ori_d[2] = des_pose[5];

        kinematics();

        inverse_kinematics(pos_d, ori_d);

        for(uint i = 1; i <= num_body; i++){
            cur_joint[i - 1] = body[i].qi;
        }

        kinematics();

        cur_pose[0] = body[num_body].re[0];
        cur_pose[1] = body[num_body].re[1];
        cur_pose[2] = body[num_body].re[2];
        cur_pose[3] = body[num_body].ori[0];
        cur_pose[4] = body[num_body].ori[1];
        cur_pose[5] = body[num_body].ori[2];

        double pos = sqrt(pow(des_pose[0] - cur_pose[0], 2) + pow(des_pose[1] - cur_pose[1], 2) + pow(des_pose[2] - cur_pose[2], 2));
        double ang_r = abs(des_pose[3] - cur_pose[3]);
        double ang_p = abs(des_pose[4] - cur_pose[4]);
        double ang_y = abs(des_pose[5] - cur_pose[5]);

        printf("[IK]pos : %f\n", pos);
        printf("[IK]ang_r : %f\t ang_p : %f\t ang_y : %f\n", ang_r, ang_p, ang_y);

        if (pos < epsilon_pos && ang_r < epsilon_ang && ang_p < epsilon_ang && ang_y < epsilon_ang){
            goal_reach = true;
            printf("[IK]iteration : %d\n", i);
            break;
        }
        else{
            goal_reach = false;
        }
    }
}

#ifdef FILEIO_H_
void RobotArm::run_inverse_kinematics_with_path_generator()
{
    file_name = "../FAR_Analysis/data/evaluation_motion_path_generator_cpp.txt";
    fp = fopen(file_name.c_str(), "w+");

    double q_init[6] = {0.7665113, -0.3214468, 2.2424687, -1.9210219, 0.2806862, 0};

    for (uint i = 1; i <= 6; i++) {
        body[i].qi = q_init[i - 1];
    }

    kinematics();

    double des_angle[3] = {1.5707963, 0, -2.094399};
    double rpy_mat[9] = {0,};
    rpy2mat(des_angle[2], des_angle[1], des_angle[0], rpy_mat);

    double r[3], theta;
    mat_to_axis_angle(body[num_body].Ae, rpy_mat, r, &theta);

    double waypoints[6*4] = {-0.208, 0.1750735, 0.07,   0,
                             -0.124, 0.2590735,	-0.014, theta,
                             -0.292, 0.2590735,	-0.014, theta,
                             -0.292, 0.0910735,	0.154,  theta,
                             -0.124, 0.0910735,	0.154,  theta,
                             -0.208, 0.1750735,	0.07,   theta};
    std::vector<double> path_x, path_y, path_z, path_theta;

    for(uint i = 0; i < 6 - 1; i++){
        path_generator(waypoints[i*4 + 0], waypoints[(i + 1)*4 + 0], 0.5, 0.1, &path_x);
        path_generator(waypoints[i*4 + 1], waypoints[(i + 1)*4 + 1], 0.5, 0.1, &path_y);
        path_generator(waypoints[i*4 + 2], waypoints[(i + 1)*4 + 2], 0.5, 0.1, &path_z);
        path_generator(waypoints[i*4 + 3], waypoints[(i + 1)*4 + 3], 0.5, 0.1, &path_theta);
    }

    double pos_d[3], ori_d[3];

    vector<double> ref_data;
    uint row = 2501;
    uint col = 26;
    load_data("../FAR_Analysis/data/evaluation_motion_path_generator_recurdyn.txt", &ref_data, "\t");

    double q_dot[6], vel[6];
    double Ri[9], Rd[9];

    for (uint indx = 0; indx < path_x.size(); indx++) {

        pos_d[0] = path_x[indx];
        pos_d[1] = path_y[indx];
        pos_d[2] = path_z[indx];

        axis_angle_to_mat(r, path_theta[indx], Ri);

        kinematics();

        mat(body[num_body].Ae, Ri, 3, 3, 3, 3, Rd);
        mat2rpy(Rd, ori_d);

        inverse_kinematics(pos_d, ori_d);

        for(uint i = 0; i < 6;i++)
        {
            q_dot[i] = ref_data[indx*col + 14 + i];
        }
        mat(J, q_dot, 6, 6, 6, vel);

        for(uint i = 1; i <= 6; i++){
            body[i].qi_dot = q_dot[i - 1];
        }

        body[num_body].re_dot[0] = vel[0];
        body[num_body].re_dot[1] = vel[1];
        body[num_body].re_dot[2] = vel[2];
        body[num_body].we[0] = vel[3];
        body[num_body].we[1] = vel[4];
        body[num_body].we[2] = vel[5];

        save_data();

        printf("Time : %.3f[s]\n", static_cast<double>(t_current));

        t_current += h;
    }

    fclose(fp);
}
#endif

void RobotArm::run_virtual_spring_damper_algorithm()
{
    file_name =  "../FAR_Analysis/data/evaluation_motion_path_generator_cpp.txt";
    fp = fopen(file_name.c_str(), "w+");

    double q_init[6] = {0.766513540000000,-0.377905660000000,2.29414790000000,-1.91624220000000,0.280680130000000,-9.76020820000000e-15};
    double q_dot_init[6] = {-0.000164439360000000,6.43344590000000e-05,0.000102933750000000,-0.000167268210000000,0.000164439360000000,2.56704870000000e-19};

    for (uint i = 1; i <= 6; i++) {
        body[i].qi = q_init[i - 1];
        body[i].qi_dot = q_dot_init[i - 1];
    }

    kinematics();

    double des_angle[3] = {1.5707963, 0, -2.094399};
    double rpy_mat[9] = {0,};
    rpy2mat(des_angle[2], des_angle[1], des_angle[0], rpy_mat);

    double r[3], theta;
    mat_to_axis_angle(body[num_body].Ae, rpy_mat, r, &theta);

    double waypoints[6*4] = {-0.208, 0.1750735, 0.07, 0,
                           -0.124,	0.2590735,	-0.014, theta,
                           -0.292,	0.2590735,	-0.014, theta,
                           -0.292,	0.0910735,	0.154, theta,
                           -0.124,	0.0910735,	0.154, theta,
                           -0.208,	0.1750735,	0.07, theta};
    std::vector<double> path_x, path_y, path_z, path_theta;

    for(uint i = 0; i < 6 - 1; i++){
        path_generator(waypoints[i*4 + 0], waypoints[(i + 1)*4 + 0], 0.5, 0.1, &path_x);
        path_generator(waypoints[i*4 + 1], waypoints[(i + 1)*4 + 1], 0.5, 0.1, &path_y);
        path_generator(waypoints[i*4 + 2], waypoints[(i + 1)*4 + 2], 0.5, 0.1, &path_z);
        path_generator(waypoints[i*4 + 3], waypoints[(i + 1)*4 + 3], 0.5, 0.1, &path_theta);
    }

    double pos_d[3], ori_d[3];
    double Ri[9], Rd[9];

    vector<double> ref_data;
    uint row = 2501;
    uint col = 26;
    load_data("../FAR_Analysis/data/evaluation_motion_path_generator_recurdyn.txt", &ref_data, "\t");

    double q_dot[6], vel[6];
    double Y[12] = {0,}, Yp[12] = {0,};
    for(uint i = 0; i < num_body; i++){
        Y[i] = body[i + 1].qi;
        Y[i + num_body] = body[i + 1].qi_dot;
    }

    for (uint indx = 0; indx < path_x.size(); indx++) {
        pos_d[0] = path_x[indx];
        pos_d[1] = path_y[indx];
        pos_d[2] = path_z[indx];


        kinematics();

        axis_angle_to_mat(r, path_theta[indx], Ri);
        mat(body[num_body].Ae, Ri, 3, 3, 3, 3, Rd);
        mat2rpy(Rd, ori_d);

        for(uint i = 0; i < num_body; i++){
            body[i + 1].qi = Y[i];
            body[i + 1].qi_dot = Y[i + num_body];
        }

        dynamics();
        jacobian();

        for(uint i = 0; i < 6;i++)
        {
            q_dot[i] = body[i+1].qi_dot;//ref_data[indx*col + 14 + i];
        }
        mat(J, q_dot, 6, 6, 6, vel);

        body[num_body].re_dot[0] = vel[0];
        body[num_body].re_dot[1] = vel[1];
        body[num_body].re_dot[2] = vel[2];
        body[num_body].we[0] = vel[3];
        body[num_body].we[1] = vel[4];
        body[num_body].we[2] = vel[5];

        double Kp = 10000; double Dp = 60;
        double Kr = 0; double Dr = 0;
        double Fd[6] = {0,}, Td[6] = {0,};
        Fd[0] = Kp*(pos_d[0] - body[num_body].re[0]) - Dp*(body[num_body].re_dot[0]);
        Fd[1] = Kp*(pos_d[1] - body[num_body].re[1]) - Dp*(body[num_body].re_dot[1]);
        Fd[2] = Kp*(pos_d[2] - body[num_body].re[2]) - Dp*(body[num_body].re_dot[2]);
        Fd[3] = Kr*(ori_d[0] - body[num_body].ori[0]) - Dr*(body[num_body].we[0]);
        Fd[4] = Kr*(ori_d[1] - body[num_body].ori[1]) - Dr*(body[num_body].we[1]);
        Fd[5] = Kr*(ori_d[2] - body[num_body].ori[2]) - Dr*(body[num_body].we[2]);

        double Jt[36] = {0,};
        for(uint i = 0; i < 6; i++){
            for(uint j = 0; j < 6; j++){
                Jt[i*6 + j] = J[j*6 + i];
            }
        }

        mat(Jt, Fd, 6, 6, 6, Td);

        double Tg[6], Ta[6];
        for(uint i = 0; i < num_body; i++){
            Tg[i] = -Q[i];
            Ta[i] = Td[i] + Tg[i];
        }
        dynamics(Ta);

        for(uint i = 0; i < num_body; i++){
            Yp[i] = body[i + 1].qi_dot;
            Yp[i + num_body] = body[i + 1].qi_ddot;
        }

        t_current = numeric->absh3(Y, Yp, t_current);
        save_data();
        printf("Time : %.3f[s]\n", static_cast<double>(t_current));
        numeric->getY_next(Y);
    }

    fclose(fp);
}

void RobotArm::run_feeding_motion()
{
    file_name = "../FAR_Analysis/data/feeding_motion_C.txt";
    fp = fopen(file_name.c_str(), "w+");

    fp1 = fopen("../FAR_Analysis/feeding_motion_data/rise_q1.txt", "w+");
    fp2 = fopen("../FAR_Analysis/feeding_motion_data/rise_q2.txt", "w+");
    fp3 = fopen("../FAR_Analysis/feeding_motion_data/rise_q3.txt", "w+");
    fp4 = fopen("../FAR_Analysis/feeding_motion_data/rise_q4.txt", "w+");
    fp5 = fopen("../FAR_Analysis/feeding_motion_data/rise_q5.txt", "w+");
    fp6 = fopen("../FAR_Analysis/feeding_motion_data/rise_q6.txt", "w+");

    std::vector<double> ready_data, step_1, step_2;
    load_data("../FAR_Analysis/feeding_motion_data/ready.txt", &ready_data, "\t");
    load_data("../FAR_Analysis/feeding_motion_data/rise_step1.txt", &step_1, "\t");
    load_data("../FAR_Analysis/feeding_motion_data/rise_step2.txt", &step_2, "\t");

    for (uint i = 1; i <= 6; i++) {
        body[i].qi = ready_data[i + 1];
    }

    kinematics();

    double rpy_mat1[9] = {0,}, rpy_mat2[9] = {0,};
    rpy2mat(step_1[13], step_1[12], step_1[11], rpy_mat1);
    rpy2mat(step_2[13], step_2[12], step_2[11], rpy_mat2);

    double r1[3], theta1, r2[3], theta2;
    mat_to_axis_angle(body[num_body].Ae, rpy_mat1, r1, &theta1);
    mat_to_axis_angle(rpy_mat1, rpy_mat2, r2, &theta2);

    const int wp_row = 5, wp_col = 4;

    vector<double> point_x, point_y, point_z, point_theta, total_time, acc_time, point_roll, point_pitch, point_yaw;
    total_time.push_back(0.0);  point_x.push_back(ready_data[0]);   point_y.push_back(ready_data[1]);   point_z.push_back(ready_data[2]);   point_roll.push_back(); point_pitch.push_back();    point_yaw.push_back();  acc_time.push_back(0.1);
    total_time.push_back(0.5);  point_x.push_back(step_1[0]);       point_y.push_back(step_1[1]);       point_z.push_back(step_1[2]);       point_roll.push_back(); point_pitch.push_back();    point_yaw.push_back();  acc_time.push_back(0.1);
    total_time.push_back(1.0);  point_x.push_back(step_2[0]);       point_y.push_back(step_2[1]);       point_z.push_back(step_2[2]);       point_roll.push_back(); point_pitch.push_back();    point_yaw.push_back();  acc_time.push_back(0.1);
    total_time.push_back(1.5);  point_x.push_back(step_2[0]);       point_y.push_back(step_2[1]);       point_z.push_back(step_2[2] + 0.1); point_roll.push_back(); point_pitch.push_back();    point_yaw.push_back();  acc_time.push_back(0.1);
    total_time.push_back(2.0);  point_x.push_back(ready_data[0]);   point_y.push_back(ready_data[1]);   point_z.push_back(ready_data[2]);   point_roll.push_back(); point_pitch.push_back();    point_yaw.push_back();  acc_time.push_back(0.1);

    for(uint8_t i = 0; i < wp_row - 1; i++){
        StructPathGenerateData path;
        path_generator(point_x[i], point_x[i + 1], total_time[i + 1] - total_time[i], acc_time[i], h, &path.path_x);
        path_generator(point_y[i], point_y[i + 1], total_time[i + 1] - total_time[i], acc_time[i], h, &path.path_y);
        path_generator(point_z[i], point_z[i + 1], total_time[i + 1] - total_time[i], acc_time[i], h, &path.path_z);
        movePath.push_back(path);
    }

    point_theta.push_back(0);
    for(uint8_t i = 0; i < wp_row - 1; i++){
        double R_init[9], R_final[9], r[3], theta;
        RobotArm::rpy2mat(point_yaw[i], point_pitch[i], point_roll[i], R_init);
        RobotArm::rpy2mat(point_yaw[i + 1], point_pitch[i + 1], point_roll[i + 1], R_final);
        RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
        memcpy(movePath[i].r, r, sizeof(double)*3);
        point_theta.push_back(theta);

//        rt_printf("r : %f, %f, %f, %f\n", r[0], r[1], r[2], theta);
    }

    for(uint8_t i = 0; i < PathData.row - 1; i++){
        path_generator(PathData.point_theta[i], PathData.point_theta[i + 1],
                PathData.total_time[i + 1] - PathData.total_time[i], PathData.acc_time[i], 0.005, &PathData.movePath[i].path_theta);

        PathData.movePath[i].data_size = PathData.movePath[i].path_x.size();
        rt_printf("section %d path size : %d\n", i, PathData.movePath[i].data_size);
    }

    for (uint indx = 0; indx < path_x.size(); indx++) {


        t_current += h;
        save_data();
        printf("Time : %.3f[s]\n", static_cast<double>(t_current));
    }

    fclose(fp);

    fclose(fp1);
    fclose(fp2);
    fclose(fp3);
    fclose(fp4);
    fclose(fp5);
    fclose(fp6);
}

#ifdef FILEIO_H_
void RobotArm::run_dynamics(){
    file_name = "../FAR_Analysis/data/evaluation_dynamics_cpp.txt";
    fp = fopen(file_name.c_str(), "w+");

    vector<double> ref_data;
    uint row = 2501;
    uint col = 20;
    load_data("../FAR_Analysis/data/evaluation_dynamics_recurdyn.txt", &ref_data, "\t");

    for (uint indx = 0; indx < row; indx++) {
        for (uint i = 0; i <= 6; i++) {
            body[i].qi = ref_data[indx*col + i + 1];
            body[i].qi_dot = ref_data[indx*col + i + 1 + 6];
        }

        kinematics();
        dynamics();

        save_data();

        printf("Time : %.3f[s]\n", static_cast<double>(t_current));

        t_current += h;
    }

    ref_data.clear();
    fclose(fp);
}
#endif

void RobotArm::kinematics()
{
    Body *body0, *body1;
    for (uint indx = 1; indx <= num_body; indx++) {
        body0 = &body[indx - 1];
        body1 = &body[indx];
        // orientation
        double *Aijpp_ptr = body1->Aijpp;
        *(Aijpp_ptr++) = cos(body1->qi);	*(Aijpp_ptr++) = -sin(body1->qi);	*(Aijpp_ptr++) = 0;
        *(Aijpp_ptr++) = sin(body1->qi);	*(Aijpp_ptr++) = cos(body1->qi);	*(Aijpp_ptr++) = 0;
        *(Aijpp_ptr++) = 0;						*(Aijpp_ptr++) = 0;						*(Aijpp_ptr++) = 1;

        mat(body0->Ai, body0->Cij, 3, 3, 3, 3, body1->Ai_Cij);
        mat(body1->Ai_Cij, body1->Aijpp, 3, 3, 3, 3, body1->Ai);

        // position
        mat(body0->Ai, body0->sijp, 3, 3, 3, body0->sij);
        for (uint i = 0; i < 3; i++){
            body1->ri[i] = body0->ri[i] + body0->sij[i];
        }
    }

    // End point
    Body *body_end = &body[num_body];
    mat(body_end->Ai, body_end->sijp, 3, 3, 3, body_end->sij);
    for (uint i = 0; i < 3; i++){
        body_end->re[i] = body_end->ri[i] + body_end->sij[i];
    }

    mat(body_end->Ai, body_end->Cij, 3, 3, 3, 3, body_end->Ae);

    mat2rpy(body_end->Ae, body_end->ori);

//    body_end->roll = atan2(body_end->Ae[2 * 3 + 1], body_end->Ae[2 * 3 + 2]);
//    body_end->pitch = atan2(-body_end->Ae[2 * 3 + 0], sqrt(pow(body_end->Ae[2 * 3 + 1], 2.0) + pow(body_end->Ae[2 * 3 + 2], 2.0)));
//    body_end->yaw = atan2(body_end->Ae[1 * 3 + 0], body_end->Ae[0 * 3 + 0]);
}

void RobotArm::inverse_kinematics(double des_pos[3], double des_ang[3]) {
    Body *body_end = &body[num_body];
    for (uint i = 0; i < 3; i++) {
        PH_pos[i] = des_pos[i] - body_end->re[i];
    }
    PH_ori[0] = des_ang[0] - body_end->ori[0];
    PH_ori[1] = des_ang[1] - body_end->ori[1];
    PH_ori[2] = des_ang[2] - body_end->ori[2];

    for (uint i = 0; i < 3; i++) {
        PH[i] = PH_pos[i];
        PH[i + 3] = PH_ori[i];
    }

#if 0
    jacobian();

    double *U, *s, *V;
    U = new double[dof * dof];
    s = new double[MIN(dof, num_body)];
    V = new double[num_body*num_body];

    numeric->svdcmp(J, dof, num_body, U, s, V);

    memset(JD, 0, sizeof(double) * num_body*dof);
    double *temp = new double[num_body*dof];
    double lamda = 1e-5;
    for (uint i = 0; i < dof; i++) {
        for (uint j = 0; j < num_body; j++) {
            for (uint k = 0; k < dof; k++) {
                temp[j * dof + k] = V[j * num_body + i] * U[k * num_body + i];
            }
        }
        for (uint j = 0; j < num_body; j++) {
            for (uint k = 0; k < dof; k++) {
                JD[j * dof + k] += (s[i] / (s[i]*s[i] +lamda*lamda))*(temp[j * dof + k]);
            }
        }
    }

    delete[] s;
    delete[] U;
    delete[] V;
    delete[] temp;


    memset(delta_q, 0, sizeof(double) * 6);
    for (uint i = 0; i < num_body; i++) {
        for (uint j = 0; j < num_body; j++) {
            delta_q[i] += JD[i * num_body + j] * PH[j];
        }
    }
#else

    int *indx = new int[6];
    double *fac = new double[6*6];
    double errmax = 0;
    int NRcount = 0;

    do{
        jacobian();

        numeric->ludcmp(J, 6, indx, 0.0, fac);
        memset(delta_q, 0, sizeof(double) * 6);
        numeric->lubksb(fac, 6, indx, PH, delta_q);

        for (uint i = 0; i < num_body; i++) {
            body[i + 1].qi += delta_q[i];
        }

        kinematics();

        for (uint i = 0; i < 3; i++) {
            PH_pos[i] = des_pos[i] - body_end->re[i];
        }
        PH_ori[0] = des_ang[0] - body_end->ori[0];
        PH_ori[1] = des_ang[1] - body_end->ori[1];
        PH_ori[2] = des_ang[2] - body_end->ori[2];

        for (uint i = 0; i < 3; i++) {
            PH[i] = PH_pos[i];
            PH[i + 3] = PH_ori[i];
        }

        errmax = PH[0];
        for(uint i = 1; i < num_body;i++){
            errmax = errmax > abs(PH[i]) ? errmax : abs(PH[i]);
        }

        NRcount++;
    }while(errmax > 1e-3 && NRcount < 5);

    printf("[IK]Err Max : %E\t : Iteration : %d\n", errmax, NRcount);

    delete[] indx;
    delete[] fac;
#endif
}

void RobotArm::jacobian()
{
    double *Jv = new double[3 * num_body];
    double *Jw = new double[3 * num_body];

    Body *body0, *body1;
    Body *body_end = &body[num_body];

    for (uint indx = 1; indx <= num_body; indx++) {
        body0 = &body[indx - 1];
        body1 = &body[indx];
        mat(body0->Cij, body1->Aijpp, 3, 3, 3, 3, body0->Cij_Aijpp);

        double *Aijpp_qi_ptr = body1->Aijpp_qi;
        *(Aijpp_qi_ptr++) = -sin(body1->qi);	*(Aijpp_qi_ptr++) = -cos(body1->qi);	*(Aijpp_qi_ptr++) = 0;
        *(Aijpp_qi_ptr++) =  cos(body1->qi);	*(Aijpp_qi_ptr++) = -sin(body1->qi);	*(Aijpp_qi_ptr++) = 0;
        *(Aijpp_qi_ptr++) = 0;						*(Aijpp_qi_ptr++) = 0;						*(Aijpp_qi_ptr) = 0;

        mat(body1->Ai_Cij, body1->Aijpp_qi, 3, 3, 3, 3, body1->Ai_Cij_Aijpp_qi);
    }

    double temp1[9] = { 0, }, temp2[3] = {0,};
    for (uint indx = 1; indx <= num_body; indx++) {
        body0 = &body[indx - 1];
        body1 = &body[indx];
        memset(body1->A6_qi, 0, sizeof(double) * 9);
        memset(body1->r6_qi, 0, sizeof(double) * 3);
        for (uint indx2 = indx; indx2 <= num_body; indx2++) {
            if (indx2 == indx) {
                for(uint i = 0; i < 9;i++){
                    body1->A6_qi[i] += body1->Ai_Cij_Aijpp_qi[i];
                }
            }
            else {
                mat(body1->A6_qi, body[indx2 - 1].Cij_Aijpp, 3, 3, 3, 3, temp1);
                memcpy(body1->A6_qi, temp1, sizeof(double) * 9);
            }
            if (indx2 < num_body) {
                mat(body1->A6_qi, body[indx2].sijp, 3, 3, 3, temp2);
                for(uint i = 0; i < 3; i++){
                    body1->r6_qi[i] += temp2[i];
                }
            }
        }

        mat(body1->A6_qi, body_end->Cij, 3, 3, 3, 3, body1->Ae_qi);
        mat(body1->A6_qi, body_end->sijp, 3, 3, 3, body1->re_qi);
        for (uint i = 0; i < 3; i++){
            body1->re_qi[i] += body1->r6_qi[i];
        }

        for (uint i = 0; i < 3; i++){
            Jv[i*num_body + indx - 1] = body1->re_qi[i];
        }
    }

    Ae_31 = body_end->Ae[6];
    Ae_32 = body_end->Ae[7];
    Ae_33 = body_end->Ae[8];
    Ae_21 = body_end->Ae[3];
    Ae_11 = body_end->Ae[0];

    roll_q_temp1 = Ae_32 * Ae_32 + Ae_33 * Ae_33;
    roll_q_temp2 = sqrt(roll_q_temp1);
    roll_q_temp3 = Ae_33 + roll_q_temp2;
    roll_q_temp4 = roll_q_temp2 * (roll_q_temp1 + Ae_33*roll_q_temp2);

    pitch_q_temp1 = sqrt(Ae_32*Ae_32 + Ae_33*Ae_33);
    pitch_q_temp2 = Ae_31 * Ae_31 + pitch_q_temp1 * pitch_q_temp1;
    pitch_q_temp3 = sqrt(pitch_q_temp2);
    pitch_q_temp4 = pitch_q_temp3 * (pitch_q_temp2 + pitch_q_temp1 * pitch_q_temp3);

    yaw_q_temp1 = Ae_21 * Ae_21 + Ae_11 * Ae_11;
    yaw_q_temp2 = sqrt(yaw_q_temp1);
    yaw_q_temp3 = Ae_11 + yaw_q_temp2;
    yaw_q_temp4 = yaw_q_temp2 * (yaw_q_temp1 + Ae_11*yaw_q_temp2);

    for (uint indx = 1; indx <= num_body; indx++) {
        body1 = &body[indx];
        body1->Ae_qi_31 = body1->Ae_qi[6];
        body1->Ae_qi_32 = body1->Ae_qi[7];
        body1->Ae_qi_33 = body1->Ae_qi[8];
        body1->Ae_qi_21 = body1->Ae_qi[3];
        body1->Ae_qi_11 = body1->Ae_qi[0];

        body1->roll_qi = (roll_q_temp3*(body1->Ae_qi_32*Ae_33 - Ae_32*body1->Ae_qi_33)) / roll_q_temp4;
        body1->pitch_qi = -((pitch_q_temp3 + pitch_q_temp1)*(body1->Ae_qi_31*pitch_q_temp1 - Ae_31 * (Ae_32*body1->Ae_qi_32 + Ae_33 * body1->Ae_qi_33)/pitch_q_temp1))/ pitch_q_temp4;
        body1->yaw_qi = (yaw_q_temp3*(body1->Ae_qi_21*Ae_11 - Ae_21*body1->Ae_qi_11)) / yaw_q_temp4;

        Jw[0 * num_body + indx - 1] = body1->roll_qi;
        Jw[1 * num_body + indx - 1] = body1->pitch_qi;
        Jw[2 * num_body + indx - 1] = body1->yaw_qi;
    }

    memcpy(J, Jv, sizeof(double) * 3 * num_body);
    memcpy(J + 3 * num_body, Jw, sizeof(double) * 3 * num_body);

    delete[] Jv;
    delete[] Jw;
}

void RobotArm::dynamics()
{
    Body *body0, *body1, *body2;
    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        body0 = &body[indx - 1];
        // velocity state
        mat(body1->Ai_Cij, body1->u_vec, 3, 3, 3, body1->Hi);
        tilde(body1->ri, body1->rit);
        mat(body1->rit, body1->Hi, 3, 3, 3, body1->Bi);
        memcpy(body1->Bi + 3, body1->Hi, sizeof(double)*3);
        for (uint i = 0; i < 6; i++){
            body1->Yih[i] = body0->Yih[i] + body1->Bi[i]*body1->qi_dot;
        }

        // cartesian velocity
        for (uint i = 0; i < 3; i++){
            for(uint j = 0; j < 3; j++){
                body1->Ti[i*6 + j] = i == j ? 1 : 0;
                body1->Ti[(i + 3)*6 + (j + 3)] = i == j ? 1 : 0;
                body1->Ti[(i + 3)*6 + j] = 0;
                body1->Ti[i*6 + (j + 3)] = -body1->rit[i*3 + j];
            }
        }

        mat(body1->Ti, body1->Yih, 6, 6, 6, body1->Yib);
        memcpy(body1->ri_dot, body1->Yib, sizeof(double)*3);
        memcpy(body1->wi, body1->Yib + 3, sizeof(double)*3);
        tilde(body1->wi, body1->wit);
        mat(body1->Ai, body1->rhoip, 3, 3, 3, body1->rhoi);
        for(uint i = 0; i < 3; i++){
            body1->ric[i] = body1->ri[i] + body1->rhoi[i];
        }
        mat(body1->wit, body1->rhoi, 3, 3, 3, body1->ric_dot);
        for (uint i = 0; i < 3; i++){
            body1->ric_dot[i] += body1->ri_dot[i];
        }

        // mass & force
        mat(body1->Ai, body1->Cii, 3, 3, 3, 3, body1->Ai_Cii);
        double temp[9] = {0,}, temp2 = 0;
        mat(body1->Ai_Cii, body1->Jip, 3, 3, 3, 3, temp);
        for(uint i = 0; i < 3; i++){
            for(uint j = 0; j < 3; j++){
                temp2 = 0;
                for(uint k = 0; k < 3; k++){
                    temp2 += temp[i*3 + k]*body1->Ai_Cii[j*3 + k];
                }
                body1->Jic[i*3 + j] = temp2;
            }
        }
        tilde(body1->ri_dot, body1->rit_dot);
        tilde(body1->ric_dot, body1->rict_dot);
        tilde(body1->ric, body1->rict);
        double temp3[9] = {0,};
        mat(body1->rict, body1->rict, 3, 3, 3, 3, temp3);
        for(uint i = 0; i < 3; i++){
            for(uint j = 0; j < 3; j++){
                body1->Mih[i*6 + j] = i == j ? body1->mi : 0;
                body1->Mih[(i + 3)*6 + j] = body1->mi*body1->rict[i*3 + j];
                body1->Mih[i*6 + (j + 3)] = -body1->mi*body1->rict[i*3 + j];
                body1->Mih[(i + 3)*6 + (j + 3)] = body1->Jic[i*3 + j] - body1->mi*temp3[i*3 + j];
            }
        }
        body1->fic[0] = 0;
        body1->fic[1] = 0;
        body1->fic[2] = body1->mi*g;
        body1->tic[0] = 0;
        body1->tic[1] = 0;
        body1->tic[2] = 0;
        double rict_dot_wi[3] = {0,};
        mat(body1->rict_dot, body1->wi, 3, 3, 3, rict_dot_wi);
        for (uint i = 0; i < 3; i++){
            body1->Qih_g[i] = body1->fic[i];
            body1->Qih_c[i] = body1->mi*rict_dot_wi[i];
            body1->Qih[i] = body1->Qih_g[i] + body1->Qih_c[i];
        }
        double rict_fic[3] = {0,}, rict_rict_dot_wi[3] = {0,}, Jic_wi[3] = {0,}, wit_Jic_wi[3] = {0,};
        mat(body1->rict, body1->fic, 3, 3, 3, rict_fic);
        mat(body1->rict, rict_dot_wi, 3, 3, 3, rict_rict_dot_wi);
        mat(body1->Jic, body1->wi, 3, 3, 3, Jic_wi);
        mat(body1->wit, Jic_wi, 3, 3, 3, wit_Jic_wi);
        for (uint i = 0; i < 3; i++){
            body1->Qih_g[i + 3] = rict_fic[i];
            body1->Qih_c[i + 3] = body1->mi*rict_rict_dot_wi[i] - wit_Jic_wi[i];
            body1->Qih[i + 3] = body1->tic[i] + body1->Qih_g[i + 3] + body1->Qih_c[i + 3];
        }

        // velocity coupling
        mat(body0->wit, body1->Hi, 3, 3, 3, body1->Hi_dot);
        double rit_dot_Hi[3] = {0,}, rit_Hi_dot[3] = {0,};
        mat(body1->rit_dot, body1->Hi, 3, 3, 3, rit_dot_Hi);
        mat(body1->rit, body1->Hi_dot, 3, 3, 3, rit_Hi_dot);
        for(uint i = 0; i < 3; i++){
            body1->Di[i] = (rit_dot_Hi[i] + rit_Hi_dot[i])*body1->qi_dot;
            body1->Di[i+3] = body1->Hi_dot[i]*body1->qi_dot;
        }

        memcpy(body1->Di_sum, body1->Di, sizeof(double)*6);
        for(uint indx2 = indx - 1; indx2 >= 1; indx2--){
            for(uint i = 0; i < 6; i++){
                body1->Di_sum[i] += body[indx2].Di[i];
            }
        }
    }

    // system EQM
    for(uint i = num_body; i >= 1;  i--){
        body1 = &body[i];
        if (i == num_body){
            memcpy(body1->Ki, body1->Mih, sizeof(double)*36);
            memcpy(body1->Li, body1->Qih, sizeof(double)*6);
            memcpy(body1->Li_g, body1->Qih_g, sizeof(double)*6);
            memcpy(body1->Li_c, body1->Qih_c, sizeof(double)*6);
        }
        else{
            body2 = &body[i + 1];
            for(uint i = 0; i < 36; i++){
                body1->Ki[i] = body1->Mih[i] + body2->Ki[i];
            }
            mat(body2->Ki, body2->Di, 6, 6, 6, body2->Ki_Di);
            for(uint i = 0; i < 6; i++){
                body1->Li[i] = body1->Qih[i] + body2->Li[i] - body2->Ki_Di[i];
                body1->Li_g[i] = body1->Qih_g[i] + body2->Li_g[i] - body2->Ki_Di[i];
                body1->Li_c[i] = body1->Qih_c[i] + body2->Li_c[i] - body2->Ki_Di[i];
            }
        }
    }

    memset(Q, 0, sizeof(double)*num_body);
    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        mat(body1->Ki, body1->Di_sum, 6, 6, 6, body1->Ki_Di_sum);
        for(uint i = 0; i < 6; i++){
            Q[indx - 1] += body1->Bi[i]*(body1->Li[i] - body1->Ki_Di_sum[i]);
        }
    }
}

void RobotArm::dynamics(double *Ta)
{
    Body *body0, *body1, *body2;
    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        body0 = &body[indx - 1];
        // velocity state
        mat(body1->Ai_Cij, body1->u_vec, 3, 3, 3, body1->Hi);
        tilde(body1->ri, body1->rit);
        mat(body1->rit, body1->Hi, 3, 3, 3, body1->Bi);
        memcpy(body1->Bi + 3, body1->Hi, sizeof(double)*3);
        for (uint i = 0; i < 6; i++){
            body1->Yih[i] = body0->Yih[i] + body1->Bi[i]*body1->qi_dot;
        }

        // cartesian velocity
        for (uint i = 0; i < 3; i++){
            for(uint j = 0; j < 3; j++){
                body1->Ti[i*6 + j] = i == j ? 1 : 0;
                body1->Ti[(i + 3)*6 + (j + 3)] = i == j ? 1 : 0;
                body1->Ti[(i + 3)*6 + j] = 0;
                body1->Ti[i*6 + (j + 3)] = -body1->rit[i*3 + j];
            }
        }

        mat(body1->Ti, body1->Yih, 6, 6, 6, body1->Yib);
        memcpy(body1->ri_dot, body1->Yib, sizeof(double)*3);
        memcpy(body1->wi, body1->Yib + 3, sizeof(double)*3);
        tilde(body1->wi, body1->wit);
        mat(body1->Ai, body1->rhoip, 3, 3, 3, body1->rhoi);
        for(uint i = 0; i < 3; i++){
            body1->ric[i] = body1->ri[i] + body1->rhoi[i];
        }
        mat(body1->wit, body1->rhoi, 3, 3, 3, body1->ric_dot);
        for (uint i = 0; i < 3; i++){
            body1->ric_dot[i] += body1->ri_dot[i];
        }

        // mass & force
        mat(body1->Ai, body1->Cii, 3, 3, 3, 3, body1->Ai_Cii);
        double temp[9] = {0,}, temp2 = 0;
        mat(body1->Ai_Cii, body1->Jip, 3, 3, 3, 3, temp);
        for(uint i = 0; i < 3; i++){
            for(uint j = 0; j < 3; j++){
                temp2 = 0;
                for(uint k = 0; k < 3; k++){
                    temp2 += temp[i*3 + k]*body1->Ai_Cii[j*3 + k];
                }
                body1->Jic[i*3 + j] = temp2;
            }
        }
        tilde(body1->ri_dot, body1->rit_dot);
        tilde(body1->ric_dot, body1->rict_dot);
        tilde(body1->ric, body1->rict);
        double temp3[9] = {0,};
        mat(body1->rict, body1->rict, 3, 3, 3, 3, temp3);
        for(uint i = 0; i < 3; i++){
            for(uint j = 0; j < 3; j++){
                body1->Mih[i*6 + j] = i == j ? body1->mi : 0;
                body1->Mih[(i + 3)*6 + j] = body1->mi*body1->rict[i*3 + j];
                body1->Mih[i*6 + (j + 3)] = -body1->mi*body1->rict[i*3 + j];
                body1->Mih[(i + 3)*6 + (j + 3)] = body1->Jic[i*3 + j] - body1->mi*temp3[i*3 + j];
            }
        }
        body1->fic[0] = 0;
        body1->fic[1] = 0;
        body1->fic[2] = body1->mi*g;
        body1->tic[0] = 0;
        body1->tic[1] = 0;
        body1->tic[2] = 0;
        double rict_dot_wi[3] = {0,};
        mat(body1->rict_dot, body1->wi, 3, 3, 3, rict_dot_wi);
        for (uint i = 0; i < 3; i++){
            body1->Qih_g[i] = body1->fic[i];
            body1->Qih_c[i] = body1->mi*rict_dot_wi[i];
            body1->Qih[i] = body1->Qih_g[i] + body1->Qih_c[i];
        }
        double rict_fic[3] = {0,}, rict_rict_dot_wi[3] = {0,}, Jic_wi[3] = {0,}, wit_Jic_wi[3] = {0,};
        mat(body1->rict, body1->fic, 3, 3, 3, rict_fic);
        mat(body1->rict, rict_dot_wi, 3, 3, 3, rict_rict_dot_wi);
        mat(body1->Jic, body1->wi, 3, 3, 3, Jic_wi);
        mat(body1->wit, Jic_wi, 3, 3, 3, wit_Jic_wi);
        for (uint i = 0; i < 3; i++){
            body1->Qih_g[i + 3] = rict_fic[i];
            body1->Qih_c[i + 3] = body1->mi*rict_rict_dot_wi[i] - wit_Jic_wi[i];
            body1->Qih[i + 3] = body1->tic[i] + body1->Qih_g[i + 3] + body1->Qih_c[i + 3];
        }

        // velocity coupling
        mat(body0->wit, body1->Hi, 3, 3, 3, body1->Hi_dot);
        double rit_dot_Hi[3] = {0,}, rit_Hi_dot[3] = {0,};
        mat(body1->rit_dot, body1->Hi, 3, 3, 3, rit_dot_Hi);
        mat(body1->rit, body1->Hi_dot, 3, 3, 3, rit_Hi_dot);
        for(uint i = 0; i < 3; i++){
            body1->Di[i] = (rit_dot_Hi[i] + rit_Hi_dot[i])*body1->qi_dot;
            body1->Di[i+3] = body1->Hi_dot[i]*body1->qi_dot;
        }

        memcpy(body1->Di_sum, body1->Di, sizeof(double)*6);
        for(uint indx2 = indx - 1; indx2 >= 1; indx2--){
            for(uint i = 0; i < 6; i++){
                body1->Di_sum[i] += body[indx2].Di[i];
            }
        }
    }

    // system EQM
    for(uint i = num_body; i >= 1;  i--){
        body1 = &body[i];
        if (i == num_body){
            memcpy(body1->Ki, body1->Mih, sizeof(double)*36);
            memcpy(body1->Li, body1->Qih, sizeof(double)*6);
            memcpy(body1->Li_g, body1->Qih_g, sizeof(double)*6);
            memcpy(body1->Li_c, body1->Qih_c, sizeof(double)*6);
        }
        else{
            body2 = &body[i + 1];
            for(uint i = 0; i < 36; i++){
                body1->Ki[i] = body1->Mih[i] + body2->Ki[i];
            }
            mat(body2->Ki, body2->Di, 6, 6, 6, body2->Ki_Di);
            for(uint i = 0; i < 6; i++){
                body1->Li[i] = body1->Qih[i] + body2->Li[i] - body2->Ki_Di[i];
                body1->Li_g[i] = body1->Qih_g[i] + body2->Li_g[i] - body2->Ki_Di[i];
                body1->Li_c[i] = body1->Qih_c[i] + body2->Li_c[i] - body2->Ki_Di[i];
            }
        }
    }

    memset(Q, 0, sizeof(double)*num_body);
    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        mat(body1->Ki, body1->Di_sum, 6, 6, 6, body1->Ki_Di_sum);
        for(uint i = 0; i < 6; i++){
            Q[indx - 1] += body1->Bi[i]*(body1->Li[i] - body1->Ki_Di_sum[i]);
            Q[indx - 1] += Ta[indx - 1];
        }
    }

    memset(M, 0, sizeof(double)*num_body*num_body);
    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        for(uint indx2 = 1; indx2 <= num_body; indx2++){
            body2 = &body[indx2];
            double Ki_Bi[6] = {0,}, temp = 0;
            if (indx == indx2){
                mat(body1->Ki, body1->Bi, 6, 6, 6, Ki_Bi);
            }
            else if(indx < indx2){
                mat(body2->Ki, body2->Bi, 6, 6, 6, Ki_Bi);
            }
            else if(indx > indx2){
                mat(body1->Ki, body2->Bi, 6, 6, 6, Ki_Bi);
            }
            temp = 0;
            for(uint i = 0; i < 6; i++){
                temp += body1->Bi[i]*Ki_Bi[i];
            }
            M[(indx - 1)*num_body + (indx2 - 1)] = temp;
        }
    }

    int *indx = new int[num_body];
    double *fac = new double[num_body*num_body];
    double *q_ddot = new double[num_body];

    numeric->ludcmp(M, static_cast<int>(num_body), indx, 0.0, fac);
    numeric->lubksb(fac, static_cast<int>(num_body), indx, Q, q_ddot);

    for(uint indx = 1; indx <= num_body; indx++){
        body[indx].qi_ddot = q_ddot[indx - 1];
    }

    delete[] indx;
    delete[] fac;
    delete[] q_ddot;
}

void RobotArm::path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path)
{
    double td = tf - ta;
    double vd = (xf - x0)/td;
    double xa = x0 + 0.5*ta*vd;
    double xd = xf - 0.5*ta*vd;

    double pos0, posf, vel0, velf, acc0, accf, ts;
    double a0, a1, a2, a3, a4, a5;

    // 가속 구간
    pos0 = x0; posf = xa; vel0 = 0; velf = vd; acc0 = 0; accf = 0; ts = ta;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

    for(double t = 0; t < ts; t += h){
        path->push_back(a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t);
    }

    // 등속 구간
    pos0 = xa; posf = xd; vel0 = vd; velf = vd; acc0 = 0; accf = 0; ts = td - ta;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

    for(double t = 0; t < ts; t += h){
        path->push_back(a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t);
    }

    // 감속 구간
    pos0 = xd; posf = xf; vel0 = vd; velf = 0; acc0 = 0; accf = 0; ts = tf - td;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

    for(double t = 0; t < ts; t += h){
        path->push_back(a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t);
    }
}

void RobotArm::save_data() {
    fprintf(fp, "%.7f\t", t_current);
    for (uint i = 1; i <= num_body; i++) {
        fprintf(fp, "%.7f\t", body[i].qi);
    }

    kinematics();

    fprintf(fp, "%.7f\t%.7f\t%.7f\t", body[num_body].re[0], body[num_body].re[1], body[num_body].re[2]);
    fprintf(fp, "%.7f\t%.7f\t%.7f\t", body[num_body].ori[0], body[num_body].ori[1], body[num_body].ori[2]);

    for (uint i = 1; i <= num_body; i++) {
        fprintf(fp, "%.7f\t", body[i].qi_dot);
    }

    fprintf(fp, "%.7f\t%.7f\t%.7f\t", body[num_body].re_dot[0], body[num_body].re_dot[1], body[num_body].re_dot[2]);
    fprintf(fp, "%.7f\t%.7f\t%.7f\t", body[num_body].we[0], body[num_body].we[1], body[num_body].we[2]);

    for (uint i = 1; i <= num_body; i++) {
        fprintf(fp, "%.7f\t", body[i].qi_ddot);
    }

    fprintf(fp, "\n");
}
