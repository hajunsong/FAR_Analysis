function mat = axis_angle_to_mat(r, theta)

    c = cos(theta);
    s = sin(theta);
    t = 1 - c;
    mag = sqrt(r(1)^2 + r(2)^2 + r(3)^2);
    x = r(1)/mag;
    y = r(2)/mag;
    z = r(3)/mag;
    
%     if theta == 0 || theta == pi
%         mat = [2*x*x - 1, 2*x*y, 2*x*z;
%             2*x*y, 2*y*y - 1, 2*y*z;
%             2*x*z, 2*y*z, 2*z*z - 1];
%     else
        mat = [t*x*x + c, t*x*y - z*s, t*x*z + y*s;
            t*x*y + z*s, t*y*y + c, t*y*z - x*s;
            t*x*z - y*s, t*y*z + x*s, t*z*z + c];
%     end

end