function dynamics_apply_force(Ta)

    global body g num_body
    
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
        body(i).Q_g = body(i).Bi'*(body(i).Li_g - body(i).Ki*temp);
        body(i).Q_c = body(i).Bi'*(body(i).Li_c - body(i).Ki*temp);
    end
    
    q_ddot = M\(Q + Ta);
    
    for i = 2 : num_body + 1
        body(i).qi_ddot = q_ddot(i - 1, 1);
    end

end