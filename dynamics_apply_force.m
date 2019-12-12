function dynamics_apply_force(Ta)

    global body g num_body des_pos path_x path_y path_z path_theta r indx h err_prev err_accum Ae
    
    body(1).wit = tilde(body(1).wi);
    body(1).Yih = zeros(6,1);
    
    for i = 2 : num_body + 1
        % velocity state
        body(i).Hi = body(i - 1).Ai*body(i - 1).Cij*body(i - 1).u_vec;
        body(i).rit = tilde(body(i).ri);
        body(i).Bi = [body(i).rit*body(i).Hi;body(i).Hi];
        body(i).Yih = body(i - 1).Yih + body(i).Bi*body(i).qi_dot;
        
        % cartesian velocity
        body(i).Ti = [eye(3) -body(i).rit; zeros(3) eye(3)];
        body(i).Yib = body(i).Ti*body(i).Yih;
        body(i).ri_dot = body(i).Yib(1:3,1);
        body(i).wi = body(i).Yib(4:6,1);
        body(i).wit = tilde(body(i).wi);
        body(i).rhoi = body(i).Ai*body(i).rhoip;
        body(i).ric = body(i).ri + body(i).rhoi;
        body(i).ric_dot = body(i).ri_dot + body(i).wit*body(i).rhoi;

        % mass & force
        body(i).Ai_Cii = body(i).Ai*body(i).Cii;
        body(i).Jic = body(i).Ai_Cii*body(i).Jip*body(i).Ai_Cii';
        body(i).rit_dot = tilde(body(i).ri_dot);
        body(i).rict_dot = tilde(body(i).ric_dot);
        body(i).rict = tilde(body(i).ric);
        body(i).Mih = [body(i).mi*eye(3) -body(i).mi*body(i).rict;
            body(i).mi*body(i).rict body(i).Jic - body(i).mi*body(i).rict*body(i).rict];
        body(i).fic = [0;0;body(i).mi*g];
        body(i).tic = [0;0;0];
        body(i).Qih = [body(i).fic + body(i).mi*body(i).rict_dot*body(i).wi;
            body(i).tic + body(i).rict*body(i).fic + body(i).mi*body(i).rict*body(i).rict_dot*body(i).wi - body(i).wit*body(i).Jic*body(i).wi];
        body(i).Qih_g = [body(i).fic;body(i).rict*body(i).fic];
        body(i).Qih_c = [body(i).mi*body(i).rict_dot*body(i).wi;
            body(i).mi*body(i).rict*body(i).rict_dot*body(i).wi - body(i).wit*body(i).Jic*body(i).wi];
        
        % velocity coupling
        body(i).Hi_dot = body(i - 1).wit*body(i).Hi;
        body(i).Di = [body(i).rit_dot*body(i).Hi + body(i).rit*body(i).Hi_dot;body(i).Hi_dot]*body(i).qi_dot;
    end
    
    % system EQM
    for i = num_body + 1 : -1 : 2
        if i == num_body + 1
            body(i).Ki = body(i).Mih;
            body(i).Li = body(i).Qih;
            body(i).Li_g = body(i).Qih_g;
            body(i).Li_c = body(i).Qih_c;
        else
            body(i).Ki = body(i).Mih + body(i + 1).Ki;
            body(i).Li = body(i).Qih + body(i + 1).Li - body(i + 1).Ki*body(i + 1).Di;
            body(i).Li_g = body(i).Qih_g + body(i + 1).Li_g - body(i + 1).Ki*body(i + 1).Di;
            body(i).Li_c = body(i).Qih_c + body(i + 1).Li_c - body(i + 1).Ki*body(i + 1).Di;
        end
    end
    
    M = zeros(num_body,num_body);
    for i = 2 : num_body + 1
        for i2 = 2 : num_body + 1
            if i == i2
                M(i - 1, i2 - 1) = body(i).Bi'*body(i).Ki*body(i).Bi;
            elseif i < i2
                M(i - 1, i2 - 1) = body(i).Bi'*body(i2).Ki*body(i2).Bi;
            elseif i > i2
                M(i - 1, i2 - 1) = body(i).Bi'*body(i).Ki*body(i2).Bi;
            end
        end
    end
    
    Q = zeros(num_body,1);
    for i = 2 : num_body + 1
        temp = zeros(6,1);
        for i2 = 2 : i
            temp = temp + body(i2).Di;
        end
        Q(i - 1,1) = body(i).Bi'*(body(i).Li - body(i).Ki*temp);
        Q_g(i-1,1) = body(i).Bi'*(body(i).Li_g - body(i).Ki*temp);
        Q_c(i-1,1) = body(i).Bi'*(body(i).Li_c - body(i).Ki*temp);
    end
    
    A = [-1,0,0;0,0,1;0,1,0];
    des_pos = [path_x(indx,1), path_y(indx,1), path_z(indx,1)]';
    Ri = axis_angle_to_mat(r, path_theta(indx,1));
    Rd = Ae*Ri;
    des_ori = mat2rpy(Rd);

    diff = des_ori - body(end).ori;
    mat = rpy2mat(diff(3), diff(2), diff(1));
    diff_ori = mat2rpy(mat);
    
    des = [des_pos;des_ori];
    
%     kinematics;
    
    J = jacobian2;
    for i = 2 : 7
        q_dot(i-1,1) = body(i).qi_dot;
    end
    vel = J*q_dot;
    body(end).re_dot = vel(1:3,1);
    body(end).we = vel(4:6,1);
    
    Kp = 0; Kd = 0;
    Op = 0; Od = 0;
    F(1,1) = Kp*(des(1) - body(end).pose(1,1)) - Kd*vel(1,1);
    F(2,1) = Kp*(des(2) - body(end).pose(2,1)) - Kd*vel(2,1);
    F(3,1) = Kp*(des(3) - body(end).pose(3,1)) - Kd*vel(3,1);
    F(4,1) = Op*(des(4) - body(end).pose(4,1)) - Od*vel(4,1);
    F(5,1) = Op*(des(5) - body(end).pose(5,1)) - Od*vel(5,1);
    F(6,1) = Op*(des(6) - body(end).pose(6,1)) - Od*vel(6,1);
%     F(4,1) = Rp*(des(4) - cur_ori(1,1)) - Rd*vel(4,1);
%     F(5,1) = Rp*(des(5) - cur_ori(2,1)) - Rd*vel(5,1);
%     F(6,1) = Rp*(des(6) - cur_ori(3,1)) - Rd*vel(6,1);
%     F(4,1) = Rp*(diff_ori(1)) - Rd*vel(4,1);
%     F(5,1) = Rp*(diff_ori(2)) - Rd*vel(5,1);
%     F(6,1) = Rp*(diff_ori(3)) - Rd*vel(6,1);
        
%     disp([F(4,1),des(4), body(end).pose(4), des(4) - body(end).pose(4,1)]);
%     fprintf('des : %E, cur : %E, err : %E, vel : %E, force : %E\n', ...
%         des(4), body(end).pose(4), des(4) - body(end).pose(4,1), vel(4,1), F(4,1));

    Tg = -Q;
    
    Td = J'*F;
    Ta = Td + Tg;
    
    q_ddot = M\(Q + Ta);
%     disp(q_ddot');
    
    for i = 2 : num_body + 1
        body(i).qi_ddot = q_ddot(i - 1, 1);
    end

end