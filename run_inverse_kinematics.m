clc; clear all; close all;

global start_time end_time body h t_current

read_data;

t_current = start_time;

ref_data = load('evaluation_motion_recurdyn.txt');
indx = 1;

fp = fopen('evaluation_motion_matlab.txt','w+');

for i = 2 : 7
    body(i).qi = ref_data(1, i + 1);
end

while t_current < end_time - h
    des = ref_data(indx, 9 : 14);
    
    kinematics;
    inverse_kinematics(des);
    
    save_data_motion(fp);
    
    t_current = t_current + h;
    indx = indx + 1;
    
    disp(t_current);
end

fclose('all');

plotting_motion;