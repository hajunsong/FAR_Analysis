function mat = rpy2mat(yaw, pitch, roll)

    R_yaw = [cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0,0,1];
    R_pitch = [cos(pitch), 0, sin(pitch);0,1,0;-sin(pitch),0,cos(pitch)];
    R_roll = [1,0,0;0,cos(roll),-sin(roll);0,sin(roll),cos(roll)];
    
    mat = R_yaw*R_pitch*R_roll;

end