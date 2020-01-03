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

    double angle, x, y, z; // variables for result
    double epsilon = 0.01; // margin to allow for rounding errors
    double epsilon2 = 0.1; // margin to distinguish between 0 and 180 degrees

    if (abs(m01 - m10) < epsilon && abs(m02 - m20) < epsilon && abs(m12 - m21) < epsilon){
        // singularity found
        // first check for identity matrix which must have +1 for all terms
        // in leading diagonal and zero in other terms
        if (abs(m01 + m10) < epsilon2 && abs(m02 + m20) < epsilon2 && abs(m12 + m21) < epsilon2 && abs(m00 + m11 + m22 - 3) < epsilon2){
            angle = 0;
            x = 1;
            y = 0;
            z = 0;
        }
        else{
            angle = M_PI;
            double xx = (m00 + 1)/2;
            double yy = (m11 + 1)/2;
            double zz = (m22 + 1)/2;
            double xy = (m01 + m10)/4;
            double xz = (m02 + m20)/4;
            double yz = (m12 + m21)/4;
            if (xx > yy && xx > zz){ // m00 is the largest diagonal term
                if (xx < epsilon){
                    x = 0;
                    y = 0.7071;
                    z = 0.7071;
                }
                else{
                    x = sqrt(xx);
                    y = xy/x;
                    z = xz/x;
                }
            }
            else if(yy > zz){ // m11 is the largest diagonal term
                if (yy < epsilon){
                    x = 0.7071;
                    y = 0;
                    z = 0.7071;
                }
                else{
                    y = sqrt(yy);
                    x = xy/y;
                    z = yz/y;
                }
            }
            else{ // m22 is the largest diagonal term so base result on this
                if (zz < epsilon){
                    x = 0.7071;
                    y = 0.7071;
                    z = 0;
                }
                else{
                    z = sqrt(zz);
                    x = xz/z;
                    y = yz/z;
                }
            }
        }
    }
    else {
        // as we have reached here there are no signularites so we can handle normally
        double s = sqrt((m21 - m12)*(m21 - m12) + (m02 - m20)*(m02 - m20) + (m10 - m01)*(m10 - m01));
        if (abs(s) < 0.001){
            s = 1;
        }
        // prevent divide by zero, should not happen if matrix is orthogonal and should be
        // caought by singularity test above, but I've left it in just in case
        angle = acos((m00 + m11 + m22 - 1)/2);
        x = (m21 - m12)/s;
        y = (m02 - m20)/s;
        z = (m10 - m01)/s;
    }

    *theta = angle;
    r[0] = x;
    r[1] = y;
    r[2] = z;
}

void RobotArm::axis_angle_to_mat(double r[], double angle, double mat[])
{
    double c = cos(angle);
    double s = sin(angle);
    double t = 1.0 - c;
    double x = r[0], y = r[1], z = r[2];
//      if axis is not already normalised then uncomment this
//    double magnitude = sqrt(x*x + y*y + z*z);
//    if (magnitude==0) {
//        return;
//    }
//    x /= magnitude;
//    y /= magnitude;
//    z /= magnitude;

    double m00, m01, m02, m10, m11, m12, m20, m21, m22;

    m00 = c + x*x*t;
    m11 = c + y*y*t;
    m22 = c + z*z*t;

    double tmp1 = x*y*t;
    double tmp2 = z*s;
    m10 = tmp1 + tmp2;
    m01 = tmp1 - tmp2;
    tmp1 = x*z*t;
    tmp2 = y*s;
    m20 = tmp1 - tmp2;
    m02 = tmp1 + tmp2;
    tmp1 = y*z*t;
    tmp2 = x*s;
    m21 = tmp1 + tmp2;
    m12 = tmp1 - tmp2;

    mat[0*3 + 0] = m00; mat[0*3 + 1] = m01; mat[0*3 + 2] = m02;
    mat[1*3 + 0] = m10; mat[1*3 + 1] = m11; mat[1*3 + 2] = m12;
    mat[2*3 + 0] = m20; mat[2*3 + 1] = m21; mat[2*3 + 2] = m22;
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
    // |  1   |    -90     |  0      |   0      |    -90     |
    // |  2   |     0      |  164.25 |   16.50  |    -90     |
    // |  3   |     180    |  170    |   0      |     0      |
    // |  4   |     90     |  65.25  |   0      |     90     |
    // |  5   |     90     |  0      |  -16.75  |     90     |
    // |  6   |     0      |  0      |   84     |     0      |

    DH[0] = -90;    DH[1] = 0;          DH[2] = 0;          DH[3] = -90;
    DH[4] = 0;      DH[5] = 0.16425;    DH[6] = 0.0165;     DH[7] = -90;
    DH[8] = 180;    DH[9] = 0.170;      DH[10] = 0;         DH[11] = 0;
    DH[12] = 90;    DH[13] = 0.06525;   DH[14] = 0;         DH[15] = 90;
    DH[16] = 90;    DH[17] = 0;         DH[18] = -0.01675;  DH[19] = 90;
    DH[20] = 0;     DH[21] = 0;         DH[22] = 0.084;     DH[23] = 0;

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

    body[0].u_vec[0] = 0; body[0].u_vec[1] = 0; body[0].u_vec[2] = 1;

    // body 1 variables
    Body::ang2mat(DH[0*4+3], DH[0*4+0], 0, body[1].Cij);
    body[1].sijp[0] = 0; body[1].sijp[1] = 0; body[1].sijp[2] = 0;

    Body::ang2mat(0, 0, 0, body[1].Cii, false);
    body[1].rhoip[0] = -0.000135874; body[1].rhoip[1] = -1.068e-12; body[1].rhoip[2] = -0.0246531;
    body[1].mi = 7.08684533226054e-002;
    body[1].Ixx = 2.15089916203442e-005;
    body[1].Iyy = 3.65070109080493e-005;
    body[1].Izz = 2.83989018839352e-005;
    body[1].Ixy = -1.24014643551512e-017;
    body[1].Iyz = -1.12222253664962e-010;
    body[1].Izx = 3.33296217281281e-008;
    body[1].Jip[0] = body[1].Ixx; body[1].Jip[1] = body[1].Ixy; body[1].Jip[2] = body[1].Izx;
    body[1].Jip[3] = body[1].Ixy; body[1].Jip[4] = body[1].Iyy; body[1].Jip[5] = body[1].Iyz;
    body[1].Jip[6] = body[1].Izx; body[1].Jip[7] = body[1].Iyz; body[1].Jip[8] = body[1].Izz;
    body[1].u_vec[0] = 0; body[1].u_vec[1] = 0; body[1].u_vec[2] = 1;

    // body 2 variables
    Body::ang2mat(DH[1*4+3], DH[1*4+0], 0, body[2].Cij);
    body[2].sijp[0] = DH[1*4+2]; body[2].sijp[1] = -DH[1*4+1]; body[2].sijp[2] = 0;

    Body::ang2mat(0, M_PI_2, M_PI_2, body[2].Cii, false);
    body[2].rhoip[0] = 0.00264336; body[2].rhoip[1] = -0.0319009; body[2].rhoip[2] = -0.000524792;
    body[2].mi = 0.233270004294732;
    body[2].Ixx = 4.08019849963512e-004;
    body[2].Iyy = 4.36730722674176e-004;
    body[2].Izz = 8.71040040349363e-005;
    body[2].Ixy = -3.23596533711006e-007;
    body[2].Iyz = -4.58645443586337e-005;
    body[2].Izx = -2.40916440855742e-006;
    body[2].Jip[0] = body[2].Ixx; body[2].Jip[1] = body[2].Ixy; body[2].Jip[2] = body[2].Izx;
    body[2].Jip[3] = body[2].Ixy; body[2].Jip[4] = body[2].Iyy; body[2].Jip[5] = body[2].Iyz;
    body[2].Jip[6] = body[2].Izx; body[2].Jip[7] = body[2].Iyz; body[2].Jip[8] = body[2].Izz;
    body[2].u_vec[0] = 0; body[2].u_vec[1] = 0; body[2].u_vec[2] = 1;

    // body 3 variables
    Body::ang2mat(DH[2*4+3], DH[2*4+0], 0, body[3].Cij);
    body[3].sijp[0] = DH[2*4+1]; body[3].sijp[1] = 0; body[3].sijp[2] = 0;

    Body::ang2mat(M_PI_2,M_PI_2,M_PI_2, body[3].Cii, false);
    body[3].rhoip[0] = 0.0668431; body[3].rhoip[1] = -4.49044e-11; body[3].rhoip[2] = -0.000574255;
    body[3].mi = 0.294733648136712;
    body[3].Ixx = 1.33438729955757e-003;
    body[3].Iyy = 1.35236609017727e-003;
    body[3].Izz = 6.10851857303522e-005;
    body[3].Ixy = -1.65934500194573e-013;
    body[3].Iyz = 5.8944693629749e-013;
    body[3].Izx = -1.46477397988517e-006;
    body[3].Jip[0] = body[3].Ixx; body[3].Jip[1] = body[3].Ixy; body[3].Jip[2] = body[3].Izx;
    body[3].Jip[3] = body[3].Ixy; body[3].Jip[4] = body[3].Iyy; body[3].Jip[5] = body[3].Iyz;
    body[3].Jip[6] = body[3].Izx; body[3].Jip[7] = body[3].Iyz; body[3].Jip[8] = body[3].Izz;
    body[3].u_vec[0] = 0; body[3].u_vec[1] = 0; body[3].u_vec[2] = 1;

    // body 4 variables
    Body::ang2mat(DH[3*4+3], DH[3*4+0], 0, body[4].Cij);
    body[4].sijp[0] = 0; body[4].sijp[1] = DH[3*4+1]; body[4].sijp[2] = 0;

    Body::ang2mat(M_PI_2,M_PI_2,-M_PI_2, body[4].Cii, false);
    body[4].rhoip[0] = 0.000488263; body[4].rhoip[1] = 0.0465912; body[4].rhoip[2] = 3.24848e-5;
    body[4].mi = 0.108749563323237;
    body[4].Ixx = 5.19451711277109e-005;
    body[4].Iyy = 2.17677195227188e-005;
    body[4].Izz = 5.3423100843467e-005;
    body[4].Ixy = -5.50565486810879e-008;
    body[4].Iyz = -4.1508751024039e-007;
    body[4].Izx = -1.72489029231613e-009;
    body[4].Jip[0] = body[4].Ixx; body[4].Jip[1] = body[4].Ixy; body[4].Jip[2] = body[4].Izx;
    body[4].Jip[3] = body[4].Ixy; body[4].Jip[4] = body[4].Iyy; body[4].Jip[5] = body[4].Iyz;
    body[4].Jip[6] = body[4].Izx; body[4].Jip[7] = body[4].Iyz; body[4].Jip[8] = body[4].Izz;
    body[4].u_vec[0] = 0; body[4].u_vec[1] = 0; body[4].u_vec[2] = 1;

    // body 5 variables
    Body::ang2mat(DH[4*4+3], DH[4*4+0], 0, body[5].Cij);
    body[5].sijp[0] = 0; body[5].sijp[1] = 0; body[5].sijp[2] = DH[4*4+2];

    Body::ang2mat(-M_PI_2, 0, 0, body[5].Cii, false);
    body[5].rhoip[0] = 0.0449512; body[5].rhoip[1] = -1.30501e-12; body[5].rhoip[2] = -0.00250684;
    body[5].mi = 0.110204790536652;
    body[5].Ixx = 5.65872649539517e-005;
    body[5].Iyy = 3.21370607982722e-005;
    body[5].Izz = 3.83261110287993e-005;
    body[5].Ixy = 3.11849020965568e-015;
    body[5].Iyz = 2.2933899377825e-006;
    body[5].Izx = 1.98601435820104e-015;
    body[5].Jip[0] = body[5].Ixx; body[5].Jip[1] = body[5].Ixy; body[5].Jip[2] = body[5].Izx;
    body[5].Jip[3] = body[5].Ixy; body[5].Jip[4] = body[5].Iyy; body[5].Jip[5] = body[5].Iyz;
    body[5].Jip[6] = body[5].Izx; body[5].Jip[7] = body[5].Iyz; body[5].Jip[8] = body[5].Izz;
    body[5].u_vec[0] = 0; body[5].u_vec[1] = 0; body[5].u_vec[2] = 1;

    // body 6 variables
    Body::ang2mat(DH[5*4+3], DH[5*4+0], 0, body[6].Cij);
    body[6].sijp[0] = 0; body[6].sijp[1] = 0; body[6].sijp[2] = DH[5*4+2];

    Body::ang2mat(M_PI, M_PI_2, 0, body[6].Cii, false);
    body[6].rhoip[0] = 4.25231e-8; body[6].rhoip[1] = 0.00049999; body[6].rhoip[2] = 0.0756101;
    body[6].mi = 1.06900256777816e-002;
    body[6].Ixx = 5.4230201398644e-007;
    body[6].Iyy = 8.27349038593228e-007;
    body[6].Izz = 6.80541319418483e-007;
    body[6].Ixy = 1.68734917731254e-012;
    body[6].Iyz = 1.25961065157397e-013;
    body[6].Izx = 3.11703037783589e-013;
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
        path_generator(waypoints[i*4 + 0], waypoints[(i + 1)*4 + 0], 0.5, 0.1, h, &path_x);
        path_generator(waypoints[i*4 + 1], waypoints[(i + 1)*4 + 1], 0.5, 0.1, h, &path_y);
        path_generator(waypoints[i*4 + 2], waypoints[(i + 1)*4 + 2], 0.5, 0.1, h, &path_z);
        path_generator(waypoints[i*4 + 3], waypoints[(i + 1)*4 + 3], 0.5, 0.1, h, &path_theta);
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
        path_generator(waypoints[i*4 + 0], waypoints[(i + 1)*4 + 0], 0.5, 0.1, h, &path_x);
        path_generator(waypoints[i*4 + 1], waypoints[(i + 1)*4 + 1], 0.5, 0.1, h, &path_y);
        path_generator(waypoints[i*4 + 2], waypoints[(i + 1)*4 + 2], 0.5, 0.1, h, &path_z);
        path_generator(waypoints[i*4 + 3], waypoints[(i + 1)*4 + 3], 0.5, 0.1, h, &path_theta);
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
    file_name = "../FAR_Analysis/feeding_motion_data/feeding_motion_C.txt";
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

    const int wp_row = 5;

    vector<double> point_x, point_y, point_z, point_theta, total_time, acc_time, point_roll, point_pitch, point_yaw, time_step;

    total_time.push_back(0.0);
    point_x.push_back(ready_data[8]);
    point_y.push_back(ready_data[9]);
    point_z.push_back(ready_data[10]);
    point_roll.push_back(ready_data[11]);
    point_pitch.push_back(ready_data[12]);
    point_yaw.push_back(ready_data[13]);
    acc_time.push_back(0.1);

    total_time.push_back(0.5);
    point_x.push_back(step_1[8]);
    point_y.push_back(step_1[9]);
    point_z.push_back(step_1[10]);
    point_roll.push_back(step_1[11]);
    point_pitch.push_back(step_1[12]);
    point_yaw.push_back(step_1[13]);
    acc_time.push_back(0.1);

    total_time.push_back(1.0);
    point_x.push_back(step_2[8]);
    point_y.push_back(step_2[9]);
    point_z.push_back(step_2[10]);
    point_roll.push_back(step_2[11]);
    point_pitch.push_back(step_2[12]);
    point_yaw.push_back(step_2[13]);
    acc_time.push_back(0.1);

    total_time.push_back(1.5);
    point_x.push_back(step_2[8]);
    point_y.push_back(step_2[9]);
    point_z.push_back(step_2[10] + 0.1);
    point_roll.push_back(step_2[11]);
    point_pitch.push_back(step_2[12]);
    point_yaw.push_back(step_2[13]);
    acc_time.push_back(0.1);

    total_time.push_back(2.0);
    point_x.push_back(ready_data[8]);
    point_y.push_back(ready_data[9]);
    point_z.push_back(ready_data[10]);
    point_roll.push_back(ready_data[11]);
    point_pitch.push_back(ready_data[12]);
    point_yaw.push_back(ready_data[13]);
    acc_time.push_back(0.1);

    for(uint8_t i = 0; i < wp_row - 1; i++){
        StructPathGenerateData path;
        path_generator(point_x[i], point_x[i + 1], total_time[i + 1] - total_time[i], acc_time[i], h, &path.path_x);
        path_generator(point_y[i], point_y[i + 1], total_time[i + 1] - total_time[i], acc_time[i], h, &path.path_y);
        path_generator(point_z[i], point_z[i + 1], total_time[i + 1] - total_time[i], acc_time[i], h, &path.path_z);
        movePath.push_back(path);
    }

    for(uint8_t i = 0; i < wp_row - 1; i++){
        double R_init[9], R_final[9], r[3], theta;
        RobotArm::rpy2mat(point_yaw[i], point_pitch[i], point_roll[i], R_init);
        RobotArm::rpy2mat(point_yaw[i + 1], point_pitch[i + 1], point_roll[i + 1], R_final);
        RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
        memcpy(movePath[i].r, r, sizeof(double)*3);
        point_theta.push_back(theta);
    }

    for(uint8_t i = 0; i < wp_row - 1; i++){
        path_generator(0, point_theta[i], total_time[i + 1] - total_time[i], acc_time[i], h, &movePath[i].path_theta);

        movePath[i].data_size = movePath[i].path_x.size();
        printf("section %d path size : %d\n", i, movePath[i].data_size);
    }

    double Ae[9];
    double Ri[9], Rd[9], des_ori[3];
    memcpy(Ae, body[num_body].Ae, sizeof(double)*9);
    for(uint i = 0; i < movePath.size() - 1; i++){
        for(uint j = 0; j < movePath[i].data_size; j++){

            printf("x : %.7f,\t y : %.7f,\t z : %.7f,\t theta : %.7f\n", movePath[i].path_x[j], movePath[i].path_y[j], movePath[i].path_z[j], movePath[i].path_theta[j]);

            kinematics();

            double des_pos[3];
            des_pos[0] = movePath[i].path_x[j];
            des_pos[1] = movePath[i].path_y[j];
            des_pos[2] = movePath[i].path_z[j];

            axis_angle_to_mat(movePath[i].r, movePath[i].path_theta[j], Ri);
            mat(body[num_body].Ae, Ri, 3, 3, 3, 3, Rd);
            mat2rpy(Rd, des_ori);

            printf("xd : %.7f,\t yd : %.7f,\t zd : %.7f,\t roll : %.7f,\t pitch : %.7f,\t yaw : %.7f\n", des_pos[0], des_pos[1], des_pos[2], des_ori[0], des_ori[1], des_ori[2]);

            inverse_kinematics(des_pos, des_ori);

            fprintf(fp1, "%.7f\t%.7f\n", t_current, body[1].qi);
            fprintf(fp2, "%.7f\t%.7f\n", t_current, body[2].qi);
            fprintf(fp3, "%.7f\t%.7f\n", t_current, body[3].qi);
            fprintf(fp4, "%.7f\t%.7f\n", t_current, body[4].qi);
            fprintf(fp5, "%.7f\t%.7f\n", t_current, body[5].qi);
            fprintf(fp6, "%.7f\t%.7f\n", t_current, body[6].qi);

            t_current += h;
            save_data();
            printf("Time : %.3f[s]\n", static_cast<double>(t_current));
        }
        memcpy(Ae, Rd, sizeof(double)*9);
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
        PH_ori[i] = des_ang[i] - body_end->ori[i];
    }

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
            PH_ori[i] = des_ang[i] - body_end->ori[i];
        }

        for (uint i = 0; i < 3; i++) {
            PH[i] = PH_pos[i];
            PH[i + 3] = PH_ori[i]*0;
        }

        errmax = PH[0];
        for(uint i = 1; i < num_body;i++){
            errmax = errmax > abs(PH[i]) ? errmax : abs(PH[i]);
        }

        NRcount++;
    }while(errmax > 1e-5 && NRcount < 10);

    if (NRcount == 10){
        errmax = 0;
    }

    printf("[IK]Err Max : %E\t : Iteration : %d\n", errmax, NRcount);

    delete[] indx;
    delete[] fac;
#endif
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

void RobotArm::jacobian()
{
    double *Jv = new double[3 * num_body];
    double *Jw = new double[3 * num_body];

    Body *body0, *body1;
    Body *body_end = &body[num_body];

    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        body1->Aijpp_qi[0] = -sin(body1->qi); body1->Aijpp_qi[1] = -cos(body1->qi); body1->Aijpp_qi[2] = 0;
        body1->Aijpp_qi[3] =  cos(body1->qi); body1->Aijpp_qi[4] = -sin(body1->qi); body1->Aijpp_qi[5] = 0;
        body1->Aijpp_qi[6] = 0; body1->Aijpp_qi[7] = 0; body1->Aijpp_qi[8] = 0;
    }

    for(uint indx = 1; indx <= num_body; indx++){
        memset(body[indx].Ae_qi, 0, sizeof(double)*9);
        memset(body[indx].re_qi, 0, sizeof(double)*3);
        for(uint indx2 = indx; indx2 <= num_body; indx2++){
            if (indx == indx2){
                mat(body[indx2].Ai_Cij, body[indx2].Aijpp_qi, 3, 3, 3, 3, body[indx2].Ai_Cij_Aijpp_qi);
                for(uint i = 0; i < 9; i++){
                    body[indx].Ae_qi[i] += body[indx2].Ai_Cij_Aijpp_qi[i];
                }
            }
            else{
                mat(body[indx].Ae_qi, body[indx2 - 1].Cij, 3, 3, 3, 3, body[indx2].Ae_qi_Cij);
                mat(body[indx2].Ae_qi_Cij, body[indx2].Aijpp, 3, 3, 3, 3, body[indx].Ae_qi_Cij_Aijpp);
                memcpy(body[indx].Ae_qi, body[indx].Ae_qi_Cij_Aijpp, sizeof(double)*9);
            }
            if (indx2 < num_body){
                mat(body[indx].Ae_qi, body[indx2].sijp, 3, 3, 3, body[indx2].Ae_qi_sijp);
                for(uint i = 0; i < 3; i++){
                    body[indx].re_qi[i] += body[indx2].Ae_qi_sijp[i];
                }
            }
        }
    }

    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        mat(body1->Ae_qi, body_end->sijp, 3, 3, 3, body1->Ae_qi_end);
        if (indx < num_body){
            for(uint i = 0; i < 3; i++){
                body1->Jvi[i] = body1->re_qi[i] + body1->Ae_qi_end[i];
            }
        }
        else{
            for(uint i = 0; i < 3; i++){
                body1->Jvi[i] = body1->Ae_qi_end[i];
            }
        }
        for (uint i = 0; i < 3; i++){
            Jv[i*num_body + indx - 1] = body1->Jvi[i];
        }
    }

#if 0
//    Ae_31 = body_end->Ae[6];
//    Ae_32 = body_end->Ae[7];
//    Ae_33 = body_end->Ae[8];
//    Ae_21 = body_end->Ae[3];
//    Ae_11 = body_end->Ae[0];

//    Ae_32_33 = Ae_32/Ae_33;
//    body[1].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[1].roll_qi_2 = ((body[1].Ae_qi[7]*Ae_33 - Ae_32*body[1].Ae_qi[8])/(pow(Ae_33, 2))); body[1].roll_qi = body[1].roll_qi_1*body[1].roll_qi_2;
//    body[2].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[2].roll_qi_2 = ((body[2].Ae_qi[7]*Ae_33 - Ae_32*body[2].Ae_qi[8])/(pow(Ae_33, 2))); body[2].roll_qi = body[2].roll_qi_1*body[2].roll_qi_2;
//    body[3].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[3].roll_qi_2 = ((body[3].Ae_qi[7]*Ae_33 - Ae_32*body[3].Ae_qi[8])/(pow(Ae_33, 2))); body[3].roll_qi = body[3].roll_qi_1*body[3].roll_qi_2;
//    body[4].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[4].roll_qi_2 = ((body[4].Ae_qi[7]*Ae_33 - Ae_32*body[4].Ae_qi[8])/(pow(Ae_33, 2))); body[4].roll_qi = body[4].roll_qi_1*body[4].roll_qi_2;
//    body[5].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[5].roll_qi_2 = ((body[5].Ae_qi[7]*Ae_33 - Ae_32*body[5].Ae_qi[8])/(pow(Ae_33, 2))); body[5].roll_qi = body[5].roll_qi_1*body[5].roll_qi_2;
//    body[6].roll_qi_1 = (1/(1 + pow(Ae_32_33, 2))); body[6].roll_qi_2 = ((body[6].Ae_qi[7]*Ae_33 - Ae_32*body[6].Ae_qi[8])/(pow(Ae_33, 2))); body[6].roll_qi = body[6].roll_qi_1*body[6].roll_qi_2;

//    Ae_32_33_2 = pow(Ae_32, 2) + pow(Ae_33, 2);
//    body[1].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[1].pitch_qi_2 = (-body[1].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[1].Ae_qi[8] + Ae_33*body[1].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[1].pitch_qi = body[1].pitch_qi_1*body[1].pitch_qi_2;
//    body[2].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[2].pitch_qi_2 = (-body[2].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[2].Ae_qi[8] + Ae_33*body[2].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[2].pitch_qi = body[2].pitch_qi_1*body[2].pitch_qi_2;
//    body[3].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[3].pitch_qi_2 = (-body[3].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[3].Ae_qi[8] + Ae_33*body[3].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[3].pitch_qi = body[3].pitch_qi_1*body[3].pitch_qi_2;
//    body[4].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[4].pitch_qi_2 = (-body[4].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[4].Ae_qi[8] + Ae_33*body[4].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[4].pitch_qi = body[4].pitch_qi_1*body[4].pitch_qi_2;
//    body[5].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[5].pitch_qi_2 = (-body[5].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[5].Ae_qi[8] + Ae_33*body[5].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[5].pitch_qi = body[5].pitch_qi_1*body[5].pitch_qi_2;
//    body[6].pitch_qi_1 = (1/(1 + pow((-Ae_31/sqrt(Ae_32_33_2)), 2))); body[6].pitch_qi_2 = (-body[6].Ae_qi[6]*sqrt(Ae_32_33_2) + Ae_31*(Ae_32*body[6].Ae_qi[8] + Ae_33*body[6].Ae_qi[7])/sqrt(Ae_32_33_2))/(Ae_32_33_2); body[6].pitch_qi = body[6].pitch_qi_1*body[6].pitch_qi_2;

//    Ae_21_11 = Ae_21/Ae_11;
//    body[1].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[1].yaw_qi_2 = ((body[1].Ae_qi[3]*Ae_11 - Ae_21*body[1].Ae_qi[0])/(pow(Ae_11, 2))); body[1].yaw_qi = body[1].yaw_qi_1*body[1].yaw_qi_2;
//    body[2].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[2].yaw_qi_2 = ((body[2].Ae_qi[3]*Ae_11 - Ae_21*body[2].Ae_qi[0])/(pow(Ae_11, 2))); body[2].yaw_qi = body[2].yaw_qi_1*body[2].yaw_qi_2;
//    body[3].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[3].yaw_qi_2 = ((body[3].Ae_qi[3]*Ae_11 - Ae_21*body[3].Ae_qi[0])/(pow(Ae_11, 2))); body[3].yaw_qi = body[3].yaw_qi_1*body[3].yaw_qi_2;
//    body[4].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[4].yaw_qi_2 = ((body[4].Ae_qi[3]*Ae_11 - Ae_21*body[4].Ae_qi[0])/(pow(Ae_11, 2))); body[4].yaw_qi = body[4].yaw_qi_1*body[4].yaw_qi_2;
//    body[5].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[5].yaw_qi_2 = ((body[5].Ae_qi[3]*Ae_11 - Ae_21*body[5].Ae_qi[0])/(pow(Ae_11, 2))); body[5].yaw_qi = body[5].yaw_qi_1*body[5].yaw_qi_2;
//    body[6].yaw_qi_1 = (1/(1 + pow(Ae_21_11, 2))); body[6].yaw_qi_2 = ((body[6].Ae_qi[3]*Ae_11 - Ae_21*body[6].Ae_qi[0])/(pow(Ae_11, 2))); body[6].yaw_qi = body[6].yaw_qi_1*body[6].yaw_qi_2;

//    body[1].Jwi[0] = body[1].roll_qi; body[1].Jwi[1] = body[1].pitch_qi; body[1].Jwi[2] = body[1].yaw_qi;
//    body[2].Jwi[0] = body[2].roll_qi; body[2].Jwi[1] = body[2].pitch_qi; body[2].Jwi[2] = body[2].yaw_qi;
//    body[3].Jwi[0] = body[3].roll_qi; body[3].Jwi[1] = body[3].pitch_qi; body[3].Jwi[2] = body[3].yaw_qi;
//    body[4].Jwi[0] = body[4].roll_qi; body[4].Jwi[1] = body[4].pitch_qi; body[4].Jwi[2] = body[4].yaw_qi;
//    body[5].Jwi[0] = body[5].roll_qi; body[5].Jwi[1] = body[5].pitch_qi; body[5].Jwi[2] = body[5].yaw_qi;
//    body[6].Jwi[0] = body[6].roll_qi; body[6].Jwi[1] = body[6].pitch_qi; body[6].Jwi[2] = body[6].yaw_qi;

#else
    double t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;
    double t11, t12, t13, t14, t15, t16, t17, t18, t19, t20;
    double t21, t22, t23, t24, t25, t26, t27, t28, t29, t30;
    double t31, t32, t33, t34, t35, t36, t37, t38, t39, t40;
    double t41, t42, t43, t44, t45, t46, t47, t48, t49, t50;
    double t51, t52, t53, t54, t55, t56, t57, t58, t59, t60;
    double t61, t62, t63, t64, t65, t66, t67;
    double A0[9], C01[9], q1, q2, q3, q4, q5, q6, C12[9], C23[9], C34[9], C45[9], C56[9], C6e[9], s12p[3], s23p[3], s34p[3], s45p[3], s56p[3], s6ep[3];

    memcpy(A0, body[0].Ai, sizeof(double)*9);
    memcpy(C01, body[0].Cij, sizeof(double)*9);

    q1 = body[1].qi;
    q2 = body[2].qi;
    q3 = body[3].qi;
    q4 = body[4].qi;
    q5 = body[5].qi;
    q6 = body[6].qi;

    memcpy(C12, body[1].Cij, sizeof(double)*9);
    memcpy(C23, body[2].Cij, sizeof(double)*9);
    memcpy(C34, body[3].Cij, sizeof(double)*9);
    memcpy(C45, body[4].Cij, sizeof(double)*9);
    memcpy(C56, body[5].Cij, sizeof(double)*9);
    memcpy(C6e, body[6].Cij, sizeof(double)*9);

    memcpy(s12p, body[1].sijp, sizeof(double)*3);
    memcpy(s23p, body[2].sijp, sizeof(double)*3);
    memcpy(s34p, body[3].sijp, sizeof(double)*3);
    memcpy(s45p, body[4].sijp, sizeof(double)*3);
    memcpy(s56p, body[5].sijp, sizeof(double)*3);
    memcpy(s6ep, body[6].sijp, sizeof(double)*3);

    t1 = A0[2*3+0] * C01[0*3+0] + A0[2*3+1] * C01[1*3+0] + A0[2*3+2] * C01[2*3+0];
    t2 = sin(q1);
    t3 = A0[2*3+0] * C01[0*3+1] + A0[2*3+1] * C01[1*3+1] + A0[2*3+2] * C01[2*3+1];
    t4 = cos(q1);
    t5 = t1 * t2 - t3 * t4;
    t1 = t1 * t4 + t2 * t3;
    t3 = t1 * C12[1*3+0] + t5 * C12[0*3+0];
    t6 = cos(q2);
    t7 = t1 * C12[1*3+1] + t5 * C12[0*3+1];
    t8 = sin(q2);
    t9 = t3 * t6 + t7 * t8;
    t3 = -t3 * t8 + t6 * t7;
    t7 = t1 * C12[1*3+2] + t5 * C12[0*3+2];
    t10 = t3 * C23[1*3+0] + t7 * C23[2*3+0] + t9 * C23[0*3+0];
    t11 = cos(q3);
    t12 = t3 * C23[1*3+1] + t7 * C23[2*3+1] + t9 * C23[0*3+1];
    t13 = sin(q3);
    t14 = t10 * t11 + t12 * t13;
    t10 = -t10 * t13 + t11 * t12;
    t3 = t3 * C23[1*3+2] + t7 * C23[2*3+2] + t9 * C23[0*3+2];
    t7 = t10 * C34[1*3+0] + t14 * C34[0*3+0] + t3 * C34[2*3+0];
    t9 = cos(q4);
    t12 = t10 * C34[1*3+1] + t14 * C34[0*3+1] + t3 * C34[2*3+1];
    t15 = sin(q4);
    t16 = t12 * t15 + t7 * t9;
    t7 = t12 * t9 - t15 * t7;
    t3 = t10 * C34[1*3+2] + t14 * C34[0*3+2] + t3 * C34[2*3+2];
    t10 = t16 * C45[0*3+0] + t3 * C45[2*3+0] + t7 * C45[1*3+0];
    t12 = cos(q5);
    t14 = t16 * C45[0*3+1] + t3 * C45[2*3+1] + t7 * C45[1*3+1];
    t17 = sin(q5);
    t18 = t10 * t12 + t14 * t17;
    t10 = t10 * t17 - t12 * t14;
    t3 = t16 * C45[0*3+2] + t3 * C45[2*3+2] + t7 * C45[1*3+2];
    t7 = -t10 * C56[1*3+0] + t18 * C56[0*3+0] + t3 * C56[2*3+0];
    t14 = cos(q6);
    t16 = -t10 * C56[1*3+1] + t18 * C56[0*3+1] + t3 * C56[2*3+1];
    t19 = sin(q6);
    t20 = t14 * t7 + t16 * t19;
    t7 = t14 * t16 - t19 * t7;
    t3 = -t10 * C56[1*3+2] + t18 * C56[0*3+2] + t3 * C56[2*3+2];
    t10 = t20 * C6e[0*3+1] + t3 * C6e[2*3+1] + t7 * C6e[1*3+1];
    t16 = A0[2*3+0] * C01[0*3+2] + A0[2*3+1] * C01[1*3+2] + A0[2*3+2] * C01[2*3+2];
    t18 = t1 * C12[0*3+0] + t16 * C12[2*3+0] - t5 * C12[1*3+0];
    t21 = t1 * C12[0*3+1] + t16 * C12[2*3+1] - t5 * C12[1*3+1];
    t22 = t18 * t6 + t21 * t8;
    t18 = t18 * t8 - t21 * t6;
    t1 = t1 * C12[0*3+2] + t16 * C12[2*3+2] - t5 * C12[1*3+2];
    t5 = t1 * C23[2*3+0] - t18 * C23[1*3+0] + t22 * C23[0*3+0];
    t16 = t1 * C23[2*3+1] - t18 * C23[1*3+1] + t22 * C23[0*3+1];
    t21 = t11 * t5 + t13 * t16;
    t5 = -t11 * t16 + t13 * t5;
    t1 = t1 * C23[2*3+2] - t18 * C23[1*3+2] + t22 * C23[0*3+2];
    t16 = t1 * C34[2*3+0] + t21 * C34[0*3+0] - t5 * C34[1*3+0];
    t23 = t1 * C34[2*3+1] + t21 * C34[0*3+1] - t5 * C34[1*3+1];
    t24 = t15 * t23 + t16 * t9;
    t16 = t15 * t16 - t23 * t9;
    t1 = t1 * C34[2*3+2] + t21 * C34[0*3+2] - t5 * C34[1*3+2];
    t23 = t1 * C45[2*3+0] - t16 * C45[1*3+0] + t24 * C45[0*3+0];
    t25 = t1 * C45[2*3+1] - t16 * C45[1*3+1] + t24 * C45[0*3+1];
    t26 = t12 * t23 + t17 * t25;
    t23 = -t12 * t25 + t17 * t23;
    t1 = t1 * C45[2*3+2] - t16 * C45[1*3+2] + t24 * C45[0*3+2];
    t25 = t1 * C56[2*3+0] - t23 * C56[1*3+0] + t26 * C56[0*3+0];
    t27 = t1 * C56[2*3+1] - t23 * C56[1*3+1] + t26 * C56[0*3+1];
    t28 = t14 * t25 + t19 * t27;
    t25 = -t14 * t27 + t19 * t25;
    t1 = t1 * C56[2*3+2] - t23 * C56[1*3+2] + t26 * C56[0*3+2];
    t27 = t1 * C6e[2*3+2] - t25 * C6e[1*3+2] + t28 * C6e[0*3+2];
    t29 = -t1 * C6e[2*3+1] + t25 * C6e[1*3+1] - t28 * C6e[0*3+1];
    t30 = 0.1e1 / t27;
    t31 = pow(t29, 2);
    t32 = pow(t30 ,2) * t31 + 0.1e1;
    t33 = t20 * C6e[0*3+2] + t3 * C6e[2*3+2] + t7 * C6e[1*3+2];
    t32 = 0.1e1 / t32;
    t32 = t30 * t32;
    t34 = t18 * C23[0*3+0] + t22 * C23[1*3+0];
    t35 = t18 * C23[0*3+1] + t22 * C23[1*3+1];
    t36 = t11 * t34 + t13 * t35;
    t34 = -t11 * t35 + t13 * t34;
    t18 = t18 * C23[0*3+2] + t22 * C23[1*3+2];
    t22 = t18 * C34[2*3+0] - t34 * C34[1*3+0] + t36 * C34[0*3+0];
    t35 = -t18 * C34[2*3+1] + t34 * C34[1*3+1] - t36 * C34[0*3+1];
    t37 = t15 * t35 - t22 * t9;
    t22 = t15 * t22 + t35 * t9;
    t18 = t18 * C34[2*3+2] - t34 * C34[1*3+2] + t36 * C34[0*3+2];
    t34 = -t18 * C45[2*3+0] + t22 * C45[1*3+0] + t37 * C45[0*3+0];
    t35 = -t18 * C45[2*3+1] + t22 * C45[1*3+1] + t37 * C45[0*3+1];
    t36 = t12 * t34 + t17 * t35;
    t34 = -t12 * t35 + t17 * t34;
    t18 = t18 * C45[2*3+2] - t22 * C45[1*3+2] - t37 * C45[0*3+2];
    t22 = t18 * C56[2*3+0] + t34 * C56[1*3+0] - t36 * C56[0*3+0];
    t35 = -t18 * C56[2*3+1] - t34 * C56[1*3+1] + t36 * C56[0*3+1];
    t37 = t14 * t22 - t19 * t35;
    t22 = t14 * t35 + t19 * t22;
    t18 = -t18 * C56[2*3+2] - t34 * C56[1*3+2] + t36 * C56[0*3+2];
    t34 = -t18 * C6e[2*3+1] - t22 * C6e[1*3+1] + t37 * C6e[0*3+1];
    t35 = -t18 * C6e[2*3+2] - t22 * C6e[1*3+2] + t37 * C6e[0*3+2];
    t36 = t21 * C34[1*3+0] + t5 * C34[0*3+0];
    t38 = t21 * C34[1*3+1] + t5 * C34[0*3+1];
    t39 = t15 * t38 + t36 * t9;
    t36 = t15 * t36 - t38 * t9;
    t5 = t21 * C34[1*3+2] + t5 * C34[0*3+2];
    t21 = t36 * C45[1*3+0] - t39 * C45[0*3+0] - t5 * C45[2*3+0];
    t38 = -t36 * C45[1*3+1] + t39 * C45[0*3+1] + t5 * C45[2*3+1];
    t40 = t12 * t21 - t17 * t38;
    t21 = t12 * t38 + t17 * t21;
    t5 = -t36 * C45[1*3+2] + t39 * C45[0*3+2] + t5 * C45[2*3+2];
    t36 = t21 * C56[1*3+0] - t40 * C56[0*3+0] + t5 * C56[2*3+0];
    t38 = -t21 * C56[1*3+1] + t40 * C56[0*3+1] - t5 * C56[2*3+1];
    t39 = t14 * t36 - t19 * t38;
    t36 = t14 * t38 + t19 * t36;
    t5 = -t21 * C56[1*3+2] + t40 * C56[0*3+2] - t5 * C56[2*3+2];
    t21 = t36 * C6e[1*3+1] - t39 * C6e[0*3+1] + t5 * C6e[2*3+1];
    t38 = -t36 * C6e[1*3+2] + t39 * C6e[0*3+2] - t5 * C6e[2*3+2];
    t40 = t16 * C45[0*3+0] + t24 * C45[1*3+0];
    t41 = t16 * C45[0*3+1] + t24 * C45[1*3+1];
    t42 = t12 * t40 + t17 * t41;
    t40 = -t12 * t41 + t17 * t40;
    t16 = t16 * C45[0*3+2] + t24 * C45[1*3+2];
    t24 = t16 * C56[2*3+0] - t40 * C56[1*3+0] + t42 * C56[0*3+0];
    t41 = t16 * C56[2*3+1] - t40 * C56[1*3+1] + t42 * C56[0*3+1];
    t43 = t14 * t24 + t19 * t41;
    t24 = -t14 * t41 + t19 * t24;
    t16 = t16 * C56[2*3+2] - t40 * C56[1*3+2] + t42 * C56[0*3+2];
    t40 = t16 * C6e[2*3+1] - t24 * C6e[1*3+1] + t43 * C6e[0*3+1];
    t41 = t16 * C6e[2*3+2] - t24 * C6e[1*3+2] + t43 * C6e[0*3+2];
    t42 = t23 * C56[0*3+0] + t26 * C56[1*3+0];
    t44 = t23 * C56[0*3+1] + t26 * C56[1*3+1];
    t45 = t14 * t42 + t19 * t44;
    t42 = -t14 * t44 + t19 * t42;
    t23 = t23 * C56[0*3+2] + t26 * C56[1*3+2];
    t26 = t23 * C6e[2*3+1] - t42 * C6e[1*3+1] + t45 * C6e[0*3+1];
    t44 = t23 * C6e[2*3+2] - t42 * C6e[1*3+2] + t45 * C6e[0*3+2];
    t46 = t25 * C6e[0*3+1] + t28 * C6e[1*3+1];
    t47 = t25 * C6e[0*3+2] + t28 * C6e[1*3+2];
    t31 = pow(t27 ,2) + t31;
    t48 = pow(t31 , (-0.3e1 / 0.2e1));
    t49 = t31 * t48;
    t1 = t1 * C6e[2*3+0] - t25 * C6e[1*3+0] + t28 * C6e[0*3+0];
    t31 = 0.1e1 / t31;
    t31 = t31 * pow(t1 ,2) + 0.1e1;
    t31 = 0.1e1 / t31;
    body[1].roll_qi = t31 * ((t20 * C6e[0*3+0] + t3 * C6e[2*3+0] + t7 * C6e[1*3+0]) * t49 + t48 * (t10 * t29 - t27 * t33) * t1);
    body[2].roll_qi = t31 * (t49 * (-t18 * C6e[2*3+0] - t22 * C6e[1*3+0] + t37 * C6e[0*3+0]) + t48 * (-t27 * t35 + t29 * t34) * t1);
    body[3].roll_qi = -t31 * (-t49 * (-t36 * C6e[1*3+0] + t39 * C6e[0*3+0] - t5 * C6e[2*3+0]) + t48 * (t21 * t29 + t27 * t38) * t1);
    body[4].roll_qi = -t31 * (-t49 * (t16 * C6e[2*3+0] - t24 * C6e[1*3+0] + t43 * C6e[0*3+0]) + t48 * (t27 * t41 - t29 * t40) * t1);
    body[5].roll_qi = t31 * (t49 * (t23 * C6e[2*3+0] - t42 * C6e[1*3+0] + t45 * C6e[0*3+0]) - t48 * (-t26 * t29 + t27 * t44) * t1);
    body[6].roll_qi = t31 * (t49 * (t25 * C6e[0*3+0] + t28 * C6e[1*3+0]) - t48 * (t27 * t47 - t29 * t46) * t1);
    t3 = A0[1*3+0] * C01[0*3+0] + A0[1*3+1] * C01[1*3+0] + A0[1*3+2] * C01[2*3+0];
    t5 = A0[1*3+0] * C01[0*3+1] + A0[1*3+1] * C01[1*3+1] + A0[1*3+2] * C01[2*3+1];
    t7 = t2 * t3 - t4 * t5;
    t3 = t2 * t5 + t3 * t4;
    t5 = t3 * C12[1*3+0] + t7 * C12[0*3+0];
    t16 = t3 * C12[1*3+1] + t7 * C12[0*3+1];
    t18 = t16 * t8 + t5 * t6;
    t5 = -t16 * t6 + t5 * t8;
    t16 = t3 * C12[1*3+2] + t7 * C12[0*3+2];
    t20 = -t16 * C23[2*3+0] - t18 * C23[0*3+0] + t5 * C23[1*3+0];
    t22 = t16 * C23[2*3+1] + t18 * C23[0*3+1] - t5 * C23[1*3+1];
    t23 = t11 * t20 - t13 * t22;
    t20 = t11 * t22 + t13 * t20;
    t5 = t16 * C23[2*3+2] + t18 * C23[0*3+2] - t5 * C23[1*3+2];
    t16 = t20 * C34[1*3+0] - t23 * C34[0*3+0] + t5 * C34[2*3+0];
    t18 = t20 * C34[1*3+1] - t23 * C34[0*3+1] + t5 * C34[2*3+1];
    t22 = t15 * t18 + t16 * t9;
    t16 = t15 * t16 - t18 * t9;
    t5 = t20 * C34[1*3+2] - t23 * C34[0*3+2] + t5 * C34[2*3+2];
    t18 = -t16 * C45[1*3+0] + t22 * C45[0*3+0] + t5 * C45[2*3+0];
    t20 = -t16 * C45[1*3+1] + t22 * C45[0*3+1] + t5 * C45[2*3+1];
    t23 = t12 * t18 + t17 * t20;
    t18 = t12 * t20 - t17 * t18;
    t5 = -t16 * C45[1*3+2] + t22 * C45[0*3+2] + t5 * C45[2*3+2];
    t16 = t18 * C56[1*3+0] + t23 * C56[0*3+0] + t5 * C56[2*3+0];
    t20 = t18 * C56[1*3+1] + t23 * C56[0*3+1] + t5 * C56[2*3+1];
    t22 = A0[0*3+0] * C01[0*3+0] + A0[0*3+1] * C01[1*3+0] + A0[0*3+2] * C01[2*3+0];
    t24 = A0[0*3+0] * C01[0*3+1] + A0[0*3+1] * C01[1*3+1] + A0[0*3+2] * C01[2*3+1];
    t25 = t2 * t24 + t22 * t4;
    t2 = t2 * t22 - t24 * t4;
    t4 = A0[0*3+0] * C01[0*3+2] + A0[0*3+1] * C01[1*3+2] + A0[0*3+2] * C01[2*3+2];
    t22 = -t2 * C12[1*3+0] + t25 * C12[0*3+0] + t4 * C12[2*3+0];
    t24 = -t2 * C12[1*3+1] + t25 * C12[0*3+1] + t4 * C12[2*3+1];
    t27 = t22 * t6 + t24 * t8;
    t22 = t22 * t8 - t24 * t6;
    t4 = -t2 * C12[1*3+2] + t25 * C12[0*3+2] + t4 * C12[2*3+2];
    t24 = -t22 * C23[1*3+0] + t27 * C23[0*3+0] + t4 * C23[2*3+0];
    t28 = -t22 * C23[1*3+1] + t27 * C23[0*3+1] + t4 * C23[2*3+1];
    t31 = t11 * t24 + t13 * t28;
    t24 = -t11 * t28 + t13 * t24;
    t4 = -t22 * C23[1*3+2] + t27 * C23[0*3+2] + t4 * C23[2*3+2];
    t28 = -t24 * C34[1*3+0] + t31 * C34[0*3+0] + t4 * C34[2*3+0];
    t36 = -t24 * C34[1*3+1] + t31 * C34[0*3+1] + t4 * C34[2*3+1];
    t37 = t15 * t36 + t28 * t9;
    t28 = t15 * t28 - t36 * t9;
    t4 = -t24 * C34[1*3+2] + t31 * C34[0*3+2] + t4 * C34[2*3+2];
    t36 = -t28 * C45[1*3+0] + t37 * C45[0*3+0] + t4 * C45[2*3+0];
    t39 = -t28 * C45[1*3+1] + t37 * C45[0*3+1] + t4 * C45[2*3+1];
    t42 = t12 * t36 + t17 * t39;
    t36 = -t12 * t39 + t17 * t36;
    t4 = -t28 * C45[1*3+2] + t37 * C45[0*3+2] + t4 * C45[2*3+2];
    t39 = -t36 * C56[1*3+0] + t4 * C56[2*3+0] + t42 * C56[0*3+0];
    t43 = -t36 * C56[1*3+1] + t4 * C56[2*3+1] + t42 * C56[0*3+1];
    t45 = t14 * t39 + t19 * t43;
    t39 = -t14 * t43 + t19 * t39;
    t4 = -t39 * C6e[1*3+0] + t45 * C6e[0*3+0] + (-t36 * C56[1*3+2] + t4 * C56[2*3+2] + t42 * C56[0*3+2]) * C6e[2*3+0];
    t43 = A0[1*3+0] * C01[0*3+2] + A0[1*3+1] * C01[1*3+2] + A0[1*3+2] * C01[2*3+2];
    t48 = t3 * C12[0*3+0] + t43 * C12[2*3+0] - t7 * C12[1*3+0];
    t49 = t3 * C12[0*3+1] + t43 * C12[2*3+1] - t7 * C12[1*3+1];
    t50 = t48 * t6 + t49 * t8;
    t48 = t48 * t8 - t49 * t6;
    t3 = t3 * C12[0*3+2] + t43 * C12[2*3+2] - t7 * C12[1*3+2];
    t7 = t3 * C23[2*3+0] - t48 * C23[1*3+0] + t50 * C23[0*3+0];
    t43 = t3 * C23[2*3+1] - t48 * C23[1*3+1] + t50 * C23[0*3+1];
    t49 = t11 * t7 + t13 * t43;
    t7 = -t11 * t43 + t13 * t7;
    t3 = t3 * C23[2*3+2] - t48 * C23[1*3+2] + t50 * C23[0*3+2];
    t43 = t3 * C34[2*3+0] + t49 * C34[0*3+0] - t7 * C34[1*3+0];
    t51 = t3 * C34[2*3+1] + t49 * C34[0*3+1] - t7 * C34[1*3+1];
    t52 = t15 * t51 + t43 * t9;
    t43 = t15 * t43 - t51 * t9;
    t3 = t3 * C34[2*3+2] + t49 * C34[0*3+2] - t7 * C34[1*3+2];
    t51 = t3 * C45[2*3+0] - t43 * C45[1*3+0] + t52 * C45[0*3+0];
    t53 = t3 * C45[2*3+1] - t43 * C45[1*3+1] + t52 * C45[0*3+1];
    t54 = t12 * t51 + t17 * t53;
    t51 = -t12 * t53 + t17 * t51;
    t3 = t3 * C45[2*3+2] - t43 * C45[1*3+2] + t52 * C45[0*3+2];
    t53 = t3 * C56[2*3+0] - t51 * C56[1*3+0] + t54 * C56[0*3+0];
    t55 = t3 * C56[2*3+1] - t51 * C56[1*3+1] + t54 * C56[0*3+1];
    t56 = t14 * t53 + t19 * t55;
    t53 = -t14 * t55 + t19 * t53;
    t3 = -t53 * C6e[1*3+0] + t56 * C6e[0*3+0] + (t3 * C56[2*3+2] - t51 * C56[1*3+2] + t54 * C56[0*3+2]) * C6e[2*3+0];
    t4 = 0.1e1 / t4;
    t55 = pow(t3 ,2) * pow(t4 ,2) + 0.1e1;
    t57 = t2 * C12[0*3+0] + t25 * C12[1*3+0];
    t58 = t2 * C12[0*3+1] + t25 * C12[1*3+1];
    t59 = t57 * t6 + t58 * t8;
    t6 = t57 * t8 - t58 * t6;
    t2 = t2 * C12[0*3+2] + t25 * C12[1*3+2];
    t8 = t2 * C23[2*3+0] + t59 * C23[0*3+0] - t6 * C23[1*3+0];
    t25 = t2 * C23[2*3+1] + t59 * C23[0*3+1] - t6 * C23[1*3+1];
    t57 = t11 * t8 + t13 * t25;
    t8 = -t11 * t25 + t13 * t8;
    t2 = t2 * C23[2*3+2] + t59 * C23[0*3+2] - t6 * C23[1*3+2];
    t6 = t2 * C34[2*3+0] + t57 * C34[0*3+0] - t8 * C34[1*3+0];
    t25 = t2 * C34[2*3+1] + t57 * C34[0*3+1] - t8 * C34[1*3+1];
    t58 = t15 * t25 + t6 * t9;
    t6 = -t15 * t6 + t25 * t9;
    t2 = t2 * C34[2*3+2] + t57 * C34[0*3+2] - t8 * C34[1*3+2];
    t8 = t2 * C45[2*3+0] + t58 * C45[0*3+0] + t6 * C45[1*3+0];
    t25 = t2 * C45[2*3+1] + t58 * C45[0*3+1] + t6 * C45[1*3+1];
    t57 = t12 * t8 + t17 * t25;
    t8 = -t12 * t25 + t17 * t8;
    t2 = t2 * C45[2*3+2] + t58 * C45[0*3+2] + t6 * C45[1*3+2];
    t6 = t2 * C56[2*3+0] + t57 * C56[0*3+0] - t8 * C56[1*3+0];
    t25 = t2 * C56[2*3+1] + t57 * C56[0*3+1] - t8 * C56[1*3+1];
    t55 = 0.1e1 / t55;
    t55 = t4 * t55;
    t58 = t48 * C23[0*3+0] + t50 * C23[1*3+0];
    t59 = t48 * C23[0*3+1] + t50 * C23[1*3+1];
    t60 = t11 * t58 + t13 * t59;
    t58 = -t11 * t59 + t13 * t58;
    t48 = t48 * C23[0*3+2] + t50 * C23[1*3+2];
    t50 = t48 * C34[2*3+0] - t58 * C34[1*3+0] + t60 * C34[0*3+0];
    t59 = t48 * C34[2*3+1] - t58 * C34[1*3+1] + t60 * C34[0*3+1];
    t61 = t15 * t59 + t50 * t9;
    t50 = t15 * t50 - t59 * t9;
    t48 = t48 * C34[2*3+2] - t58 * C34[1*3+2] + t60 * C34[0*3+2];
    t58 = t48 * C45[2*3+0] - t50 * C45[1*3+0] + t61 * C45[0*3+0];
    t59 = t48 * C45[2*3+1] - t50 * C45[1*3+1] + t61 * C45[0*3+1];
    t60 = t12 * t58 + t17 * t59;
    t58 = -t12 * t59 + t17 * t58;
    t48 = t48 * C45[2*3+2] - t50 * C45[1*3+2] + t61 * C45[0*3+2];
    t50 = t48 * C56[2*3+0] - t58 * C56[1*3+0] + t60 * C56[0*3+0];
    t59 = t48 * C56[2*3+1] - t58 * C56[1*3+1] + t60 * C56[0*3+1];
    t61 = t22 * C23[0*3+0] + t27 * C23[1*3+0];
    t62 = t22 * C23[0*3+1] + t27 * C23[1*3+1];
    t63 = t11 * t61 + t13 * t62;
    t11 = -t11 * t62 + t13 * t61;
    t13 = t22 * C23[0*3+2] + t27 * C23[1*3+2];
    t22 = -t11 * C34[1*3+0] + t13 * C34[2*3+0] + t63 * C34[0*3+0];
    t27 = -t11 * C34[1*3+1] + t13 * C34[2*3+1] + t63 * C34[0*3+1];
    t61 = t15 * t27 + t22 * t9;
    t22 = t15 * t22 - t27 * t9;
    t11 = -t11 * C34[1*3+2] + t13 * C34[2*3+2] + t63 * C34[0*3+2];
    t13 = t11 * C45[2*3+0] - t22 * C45[1*3+0] + t61 * C45[0*3+0];
    t27 = t11 * C45[2*3+1] - t22 * C45[1*3+1] + t61 * C45[0*3+1];
    t62 = t12 * t13 + t17 * t27;
    t13 = t12 * t27 - t13 * t17;
    t11 = t11 * C45[2*3+2] - t22 * C45[1*3+2] + t61 * C45[0*3+2];
    t22 = t11 * C56[2*3+0] + t13 * C56[1*3+0] + t62 * C56[0*3+0];
    t27 = t11 * C56[2*3+1] + t13 * C56[1*3+1] + t62 * C56[0*3+1];
    t61 = t49 * C34[1*3+0] + t7 * C34[0*3+0];
    t63 = t49 * C34[1*3+1] + t7 * C34[0*3+1];
    t64 = t15 * t63 + t61 * t9;
    t61 = t15 * t61 - t63 * t9;
    t7 = t49 * C34[1*3+2] + t7 * C34[0*3+2];
    t49 = t61 * C45[1*3+0] - t64 * C45[0*3+0] - t7 * C45[2*3+0];
    t63 = -t61 * C45[1*3+1] + t64 * C45[0*3+1] + t7 * C45[2*3+1];
    t65 = t12 * t49 - t17 * t63;
    t49 = t12 * t63 + t17 * t49;
    t7 = -t61 * C45[1*3+2] + t64 * C45[0*3+2] + t7 * C45[2*3+2];
    t61 = t49 * C56[1*3+0] - t65 * C56[0*3+0] + t7 * C56[2*3+0];
    t63 = -t49 * C56[1*3+1] + t65 * C56[0*3+1] - t7 * C56[2*3+1];
    t64 = t24 * C34[0*3+0] + t31 * C34[1*3+0];
    t66 = t24 * C34[0*3+1] + t31 * C34[1*3+1];
    t67 = t15 * t66 + t64 * t9;
    t9 = t15 * t64 - t66 * t9;
    t15 = t24 * C34[0*3+2] + t31 * C34[1*3+2];
    t24 = -t15 * C45[2*3+0] - t67 * C45[0*3+0] + t9 * C45[1*3+0];
    t31 = t15 * C45[2*3+1] + t67 * C45[0*3+1] - t9 * C45[1*3+1];
    t64 = t12 * t24 - t17 * t31;
    t24 = t12 * t31 + t17 * t24;
    t9 = t15 * C45[2*3+2] + t67 * C45[0*3+2] - t9 * C45[1*3+2];
    t15 = t24 * C56[1*3+0] - t64 * C56[0*3+0] + t9 * C56[2*3+0];
    t31 = t24 * C56[1*3+1] - t64 * C56[0*3+1] + t9 * C56[2*3+1];
    t7 = t55 * ((t14 * t61 - t19 * t63) * C6e[0*3+0] - (t14 * t63 + t19 * t61) * C6e[1*3+0] + (t49 * C56[1*3+2] - t65 * C56[0*3+2] + t7 * C56[2*3+2]) * C6e[2*3+0] - ((t14 * t15 + t19 * t31) * C6e[0*3+0] - (-t14 * t31 + t15 * t19) * C6e[1*3+0] - (-t24 * C56[1*3+2] + t64 * C56[0*3+2] - t9 * C56[2*3+2]) * C6e[2*3+0]) * t3 * t4);
    t9 = t43 * C45[0*3+0] + t52 * C45[1*3+0];
    t15 = t43 * C45[0*3+1] + t52 * C45[1*3+1];
    t24 = t12 * t9 + t15 * t17;
    t9 = -t12 * t15 + t17 * t9;
    t15 = t43 * C45[0*3+2] + t52 * C45[1*3+2];
    t31 = t15 * C56[2*3+0] + t24 * C56[0*3+0] - t9 * C56[1*3+0];
    t43 = t15 * C56[2*3+1] + t24 * C56[0*3+1] - t9 * C56[1*3+1];
    t49 = t28 * C45[0*3+0] + t37 * C45[1*3+0];
    t52 = t28 * C45[0*3+1] + t37 * C45[1*3+1];
    t61 = t12 * t49 + t17 * t52;
    t12 = -t12 * t52 + t17 * t49;
    t17 = t28 * C45[0*3+2] + t37 * C45[1*3+2];
    t28 = -t12 * C56[1*3+0] + t17 * C56[2*3+0] + t61 * C56[0*3+0];
    t37 = -t12 * C56[1*3+1] + t17 * C56[2*3+1] + t61 * C56[0*3+1];
    t9 = t55 * ((t14 * t31 + t19 * t43) * C6e[0*3+0] - (-t14 * t43 + t19 * t31) * C6e[1*3+0] + (t15 * C56[2*3+2] + t24 * C56[0*3+2] - t9 * C56[1*3+2]) * C6e[2*3+0] - ((t14 * t28 + t19 * t37) * C6e[0*3+0] - (-t14 * t37 + t19 * t28) * C6e[1*3+0] + (-t12 * C56[1*3+2] + t17 * C56[2*3+2] + t61 * C56[0*3+2]) * C6e[2*3+0]) * t3 * t4);
    t12 = t51 * C56[0*3+0] + t54 * C56[1*3+0];
    t15 = t51 * C56[0*3+1] + t54 * C56[1*3+1];
    t17 = t36 * C56[0*3+0] + t42 * C56[1*3+0];
    t24 = t36 * C56[0*3+1] + t42 * C56[1*3+1];
    t12 = t55 * ((t12 * t14 + t15 * t19) * C6e[0*3+0] - (t12 * t19 - t14 * t15) * C6e[1*3+0] + (t51 * C56[0*3+2] + t54 * C56[1*3+2]) * C6e[2*3+0] + (-(t14 * t17 + t19 * t24) * C6e[0*3+0] + (-t14 * t24 + t17 * t19) * C6e[1*3+0] - (t36 * C56[0*3+2] + t42 * C56[1*3+2]) * C6e[2*3+0]) * t3 * t4);
    t15 = t55 * (-t53 * C6e[0*3+0] - t56 * C6e[1*3+0] + (t39 * C6e[0*3+0] + t45 * C6e[1*3+0]) * t3 * t4);
    body[1].pitch_qi = -t55 * (-((t14 * t6 + t19 * t25) * C6e[0*3+0] + (t14 * t25 - t19 * t6) * C6e[1*3+0] + (t2 * C56[2*3+2] + t57 * C56[0*3+2] - t8 * C56[1*3+2]) * C6e[2*3+0]) * t3 * t4 + (t14 * t16 + t19 * t20) * C6e[0*3+0] - (-t14 * t20 + t16 * t19) * C6e[1*3+0] + (t18 * C56[1*3+2] + t23 * C56[0*3+2] + t5 * C56[2*3+2]) * C6e[2*3+0]);
    body[2].pitch_qi = -t55 * ((t14 * t50 + t19 * t59) * C6e[0*3+0] - (-t14 * t59 + t19 * t50) * C6e[1*3+0] + (t48 * C56[2*3+2] - t58 * C56[1*3+2] + t60 * C56[0*3+2]) * C6e[2*3+0] - ((t14 * t22 + t19 * t27) * C6e[0*3+0] + (t14 * t27 - t19 * t22) * C6e[1*3+0] + (t11 * C56[2*3+2] + t13 * C56[1*3+2] + t62 * C56[0*3+2]) * C6e[2*3+0]) * t3 * t4);
    body[3].pitch_qi = -t7;
    body[4].pitch_qi = -t9;
    body[5].pitch_qi = -t12;
    body[6].pitch_qi = t15;

    body[1].yaw_qi = -t32 * (t33 * t29 * t30 + t10);
    body[2].yaw_qi = -t32 * (t35 * t29 * t30 + t34);
    body[3].yaw_qi = -t32 * (t38 * t29 * t30 - t21);
    body[4].yaw_qi = -t32 * (t41 * t29 * t30 + t40);
    body[5].yaw_qi = -t32 * (t44 * t29 * t30 + t26);
    body[6].yaw_qi = -t32 * (t47 * t29 * t30 + t46);

    body[1].Jwi[0] = body[1].roll_qi; body[1].Jwi[1] = body[1].pitch_qi; body[1].Jwi[2] = body[1].yaw_qi;
    body[2].Jwi[0] = body[2].roll_qi; body[2].Jwi[1] = body[2].pitch_qi; body[2].Jwi[2] = body[2].yaw_qi;
    body[3].Jwi[0] = body[3].roll_qi; body[3].Jwi[1] = body[3].pitch_qi; body[3].Jwi[2] = body[3].yaw_qi;
    body[4].Jwi[0] = body[4].roll_qi; body[4].Jwi[1] = body[4].pitch_qi; body[4].Jwi[2] = body[4].yaw_qi;
    body[5].Jwi[0] = body[5].roll_qi; body[5].Jwi[1] = body[5].pitch_qi; body[5].Jwi[2] = body[5].yaw_qi;
    body[6].Jwi[0] = body[6].roll_qi; body[6].Jwi[1] = body[6].pitch_qi; body[6].Jwi[2] = body[6].yaw_qi;
#endif

    for(uint indx = 1; indx <= num_body; indx++){
        body1 = &body[indx];
        for (uint i = 0; i < 3; i++){
            Jw[i*num_body + indx - 1] = body1->Jwi[i];
        }
    }

    memcpy(J, Jv, sizeof(double) * 3 * num_body);
    memcpy(J + 3 * num_body, Jw, sizeof(double) * 3 * num_body);

    delete[] Jv;
    delete[] Jw;
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
