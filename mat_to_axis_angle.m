function [r,theta] = mat_to_axis_angle(mat)
    
    m00 = mat(1,1); m01 = mat(1,2); m02 = mat(1,3);
    m10 = mat(2,1); m11 = mat(2,2); m12 = mat(2,3);
    m20 = mat(3,1); m21 = mat(3,2); m22 = mat(3,3);

    theta = acos((m00 + m11 + m22 - 1)/2);
%     if theta == 0
%         x = sqrt((m00 + 1)/2);
%         y = -(m01 + m10)/(4*x);
%         z = -(m01 + m20)/(4*x);
%     elseif theta == pi
%         x = sqrt((m00 + 1)/2);
%         y = (m01 + m10)/(4*x);
%         z = (m01 + m20)/(4*x);
%     else
        x = (m21 - m12)/sqrt((m21 - m12)^2+(m02 - m20)^2+(m10 - m01)^2);
        y = (m02 - m20)/sqrt((m21 - m12)^2+(m02 - m20)^2+(m10 - m01)^2);
        z = (m10 - m01)/sqrt((m21 - m12)^2+(m02 - m20)^2+(m10 - m01)^2);
%     end
    r = [x;y;z];

end