function Y_next = analysis(t_span, Y)

    global num_body body Yp
    
    for indx = 2 : num_body + 1
        body(indx).qi = Y(indx - 1);
        body(indx).qi_dot = Y(indx + num_body - 1);
    end

    kinematics;
    dynamics;
    
    Yp = [body.qi_dot, body.qi_ddot]';
    Y_next = Yp;
end