clc; clear all; close all;
 
global start_time t_current end_time h body num_body

read_data;

t_current = start_time;

ref_data = load('evaluation_dynamics_recurdyn.txt');
   
i = 1;

fp = fopen('evaluation_dynamics_matlab.txt','w+');

while t_current <= end_time - h

for indx = 2 : num_body + 1
    body(indx).qi = ref_data(i, indx + 1);
    body(indx).qi_dot = ref_data(i, indx + 7);
end

    kinematics;
    dynamics;
    
    i = i + 1;
    save_data_dynamics(fp);
    t_current = t_current + h;
    disp(t_current);
end

fclose('all');

plotting_dynamics;