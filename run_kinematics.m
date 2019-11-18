clc; clear all; close all;

global start_time end_time body h t_current

read_data;

t_current = start_time;

ref_data = load('evaluation_motion_recurdyn.txt');
indx = 1;

fp = fopen('evaluation_motion_matlab.txt','w+');

while t_current < end_time - h
    for i = 2 : 7
        body(i).qi = ref_data(indx, i + 1);
    end
    
    kinematics;
    
    save_data_motion(fp);
    
    t_current = t_current + h;
    indx = indx + 1;
    
    disp(t_current);
end

fclose('all');

plotting_motion;