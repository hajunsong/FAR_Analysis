clc; clear all; close all;

global start_time end_time body h t_current

read_data;

t_current = start_time;

ref_data = load('evaluation_motion_path_generator_recurdyn.txt');

fp = fopen('evaluation_motion_path_generator_matlab.txt','w+');

q_init = ref_data(1,3:8);

for i = 2 : 7
    body(i).qi = ref_data(1, i + 1);
end

kinematics;

des_angle = [1.5707963848748 -1.50573902647842e-08 -2.09439897358979];
rpy_mat = rpy2mat(des_angle(3), des_angle(2), des_angle(1));
[r,theta] = mat_to_axis_angle(body(7).Ae'*rpy_mat);
Ae = body(7).Ae;

waypoints = [
    -0.208, 0.1750735, 0.07, 0;
    -0.124,	0.2590735,	-0.014, theta;
    -0.292,	0.2590735,	-0.014, theta;
    -0.292,	0.0910735,	0.154, theta;
    -0.124,	0.0910735,	0.154, theta;
    -0.208,	0.1750735,	0.07, theta];

path_x = [];
path_y = [];
path_z = [];
path_theta = [];
       
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

for indx = 1 : size(path_x, 1)
    des_pos = [path_x(indx,1);path_y(indx,1);path_z(indx,1)];
    
    kinematics;    
    Ri = axis_angle_to_mat(r, path_theta(indx,1));
    Rd = Ae*Ri;
    des_ori = mat2rpy(Rd);

    inverse_kinematics([des_pos;des_ori]);
    
    q_dot = ref_data(indx, 15:20)';
    
    J = jacobian2;
    vel = J*q_dot;
    for i = 2 : 7
        body(i).qi_dot = q_dot(i - 1);
    end
    body(7).re_dot = vel(1:3,1);
    body(7).we = vel(4:6,1);
    
    save_data_motion(fp);
    
    t_current = t_current + h;
    
    disp(t_current);
end

fclose('all');

data = load('evaluation_motion_path_generator_matlab.txt');

plotting_motion(ref_data, data);