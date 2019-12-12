clc; clear all; close all;

global start_time end_time body h t_current Y Yp intcount num_body Q Q_g

format long g

read_data;

t_current = start_time;

ref_data = load('evaluation_motion_path_generator_recurdyn.txt');

fp = fopen('evaluation_vsd.txt','w+');

q_init = ref_data(1,3:8);
q_dot_init = ref_data(1,15:20);

for i = 2 : num_body + 1
    body(i).qi = q_init(i - 1);
    body(i).qi_dot = q_dot_init(i - 1);
end

kinematics;

path_x = [];
path_y = [];
path_z = [];
path_theta = [];

Ae = body(end).Ae;

des_angle = [1.5707963, 0, -2.094399];
rpy_mat = rpy2mat(des_angle(3), des_angle(2), des_angle(1));
[r,theta] = mat_to_axis_angle(Ae'*rpy_mat);

waypoints = [
    -0.208, 0.1750735, 0.07, 0;
    -0.124,	0.2590735,	-0.014, theta;
    -0.292,	0.2590735,	-0.014, theta;
    -0.292,	0.0910735,	0.154, theta;
    -0.124,	0.0910735,	0.154, theta;
    -0.208,	0.1750735,	0.07, theta];

for i = 1 : size(waypoints,1)-1
    x0 = waypoints(i,1);
    xf = waypoints(i+1,1);
    tf = 0.5;
    ta = 0.1;
    td = tf - ta;
    vd = (xf - x0)/td;
    xa = x0 + 0.5*ta*vd;
    xd = xf - 0.5*ta*vd;
    
    path_x_acc = path_generator(x0, xa, 0, vd, 0, 0, ta, h);
    path_x_const = path_generator(xa, xd, vd, vd, 0, 0, td - ta, h);
    path_x_dec = path_generator(xd, xf, vd, 0, 0, 0, tf - td, h);
    if size(path_x,1) > 1
        path_x = [path_x(1:end-1,:);path_x_acc(1:end-1,:);path_x_const;path_x_dec(2:end,:)];
    else
        path_x = [path_x_acc(1:end-1,:);path_x_const;path_x_dec(2:end,:)];
    end
    
    x0 = waypoints(i,2);
    xf = waypoints(i+1,2);
    tf = 0.5;
    ta = 0.1;
    td = tf - ta;
    vd = (xf - x0)/td;
    xa = x0 + 0.5*ta*vd;
    xd = xf - 0.5*ta*vd;
    
    path_y_acc = path_generator(x0, xa, 0, vd, 0, 0, ta, h);
    path_y_const = path_generator(xa, xd, vd, vd, 0, 0, td - ta, h);
    path_y_dec = path_generator(xd, xf, vd, 0, 0, 0, tf - td, h);
    if size(path_y,1) > 1
        path_y = [path_y(1:end-1,:);path_y_acc(1:end-1,:);path_y_const;path_y_dec(2:end,:)];
    else
        path_y = [path_y_acc(1:end-1,:);path_y_const;path_y_dec(2:end,:)];
    end
    
    x0 = waypoints(i,3);
    xf = waypoints(i+1,3);
    tf = 0.5;
    ta = 0.1;
    td = tf - ta;
    vd = (xf - x0)/td;
    xa = x0 + 0.5*ta*vd;
    xd = xf - 0.5*ta*vd;
    
    path_z_acc = path_generator(x0, xa, 0, vd, 0, 0, ta, h);
    path_z_const = path_generator(xa, xd, vd, vd, 0, 0, td - ta, h);
    path_z_dec = path_generator(xd, xf, vd, 0, 0, 0, tf - td, h);
    if size(path_z,1) > 1
        path_z = [path_z(1:end-1,:);path_z_acc(1:end-1,:);path_z_const;path_z_dec(2:end,:)];
    else
        path_z = [path_z_acc(1:end-1,:);path_z_const;path_z_dec(2:end,:)];
    end
    
    x0 = waypoints(i,4);
    xf = waypoints(i+1,4);
    tf = 0.5;
    ta = 0.1;
    td = tf - ta;
    vd = (xf - x0)/td;
    xa = x0 + 0.5*ta*vd;
    xd = xf - 0.5*ta*vd;
    
    path_theta_acc = path_generator(x0, xa, 0, vd, 0, 0, ta, h);
    path_theta_const = path_generator(xa, xd, vd, vd, 0, 0, td - ta, h);
    path_theta_dec = path_generator(xd, xf, vd, 0, 0, 0, tf - td, h);
    if size(path_theta,1) > 1
        path_theta = [path_theta(1:end-1,:);path_theta_acc(1:end-1,:);path_theta_const;path_theta_dec(2:end,:)];
    else
        path_theta = [path_theta_acc(1:end-1,:);path_theta_const;path_theta_dec(2:end,:)];
    end
end

intcount = 1;
define_Y_vector;
indx = 1;
q_dot = zeros(6,1);

err_prev = zeros(6,1);

while t_current < end_time
    des_pos = [path_x(500,1), path_y(500,1), path_z(500,1)]';
    Ri = axis_angle_to_mat(r, path_theta(500,1));
    
    Y2qdq;
    
    kinematics;
    
    Rd = Ae*Ri;
    des_ori = mat2rpy(Rd);
    
    dynamics;
    
    J = jacobian2;
    for i = 2 : 7
        q_dot(i-1,1) = body(i).qi_dot;
    end
    vel = J*q_dot;
    body(end).re_dot = vel(1:3,1);
    body(end).we = vel(4:6,1);
    
    diff_A = Rd - Ae;
    diff_ang = mat2rpy(diff_A);
    
    Kp = 1000; Dp = 15;
    Kr = 1000; Dr = 15;
    F(1,1) = Kp*(des_pos(1) - body(end).re(1,1)) - Dp*body(end).re_dot(1,1);
    F(2,1) = Kp*(des_pos(2) - body(end).re(2,1)) - Dp*body(end).re_dot(2,1);
    F(3,1) = Kp*(des_pos(3) - body(end).re(3,1)) - Dp*body(end).re_dot(3,1);
    F(4,1) = Kr*(des_ori(1) - body(end).ori(1,1)) - Dr*body(end).we(1,1);
    F(5,1) = Kr*(des_ori(2) - body(end).ori(2,1)) - Dr*body(end).we(2,1);
    F(6,1) = Kr*(des_ori(3) - body(end).ori(3,1)) - Dr*body(end).we(3,1);
%     T(1,1) = Kr*(diff_ang(1)) - Dr*body(end).we(1,1);
%     T(2,1) = Kr*(diff_ang(2)) - Dr*body(end).we(2,1);
%     T(3,1) = Kr*(diff_ang(3)) - Dr*body(end).we(3,1);
%     T(1,1) = -Dr*body(end).we(1,1);
%     T(2,1) = -Dr*body(end).we(2,1);
%     T(3,1) = -Dr*body(end).we(3,1);
    
    Tg = -Q;
    Td = J'*F;
    Ta = Td + Tg;
    
    dynamics_apply_force(Ta);
    dqddq2Yp;
    
    if intcount == 1 || intcount == 5 || intcount >= 7
        save_data_motion(fp);
        indx = indx + 1;
        disp(t_current);
    end
    
    [Y_next, t_next] = absh3(t_current, Y, Yp, h);
    Y = Y_next;
    t_current = t_next;
end

fclose('all');

plotting_motion;