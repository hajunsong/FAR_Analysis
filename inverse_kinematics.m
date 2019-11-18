function inverse_kinematics(des)

    global body NRcount
    
    PH_pos = des(1:3,1) - body(end).re;
    PH_ori = des(4:6,1) - body(end).ori;
    PH = [PH_pos;PH_ori];
        
    NRcount = 1;
    err_tol = 1e-7;
    
    while 1
        if NRcount >= 10; break; end
        
        J = jacobian2;

        delta_q = J\PH;

        for i = 2 : 7
            body(i).qi = body(i).qi + delta_q(i - 1);
        end

        kinematics;

        PH_pos = des(1:3,1) - body(end).re;
        PH_ori = des(4:6,1) - body(end).ori;
        PH = [PH_pos;PH_ori];

        err_max = max(abs(PH));

        NRcount = NRcount + 1;
        
        if err_max < err_tol; break; end
    end
    
    disp(NRcount)

end