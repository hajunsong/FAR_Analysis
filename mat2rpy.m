function angle = mat2rpy(mat)

    roll = atan2(mat(3,2), mat(3,3));
    pitch = atan2(-mat(3,1),sqrt(mat(3,2)^2 + mat(3,3)^2));
    yaw = atan2(mat(2,1), mat(1,1));
    
    angle = [roll;pitch;yaw];

end