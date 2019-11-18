function path = path_generator(pos0, posf, vel0, veld, acc0, accf, tf, h)

    t = (0 : h : tf)';
    path = zeros(size(tf,1),3);

    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*veld + 12*vel0)*tf - (3*acc0 - accf)*tf^2)/(2*tf^3);
    a4 = (30*(pos0 - posf) + (14*veld + 16*vel0)*tf + (3*acc0 - 2*accf)*tf^2)/(2*tf^4);
    a5 = (12*(posf - pos0) - (6*veld + 6*vel0)*tf - (acc0 - accf)*tf^2)/(2*tf^5);
    
    for i = 1 : size(t, 1)
        path(i,1) = a0 + a1*t(i) + a2*t(i)^2 + a3*t(i)^3 + a4*t(i)^4 + a5*t(i)^5;
        path(i,2) = a1 + 2*a2*t(i) + 3*a3*t(i)^2 + 4*a4*t(i)^3 + 5*a5*t(i)^4;
        path(i,3) = 2*a2 + 6*a3*t(i) + 12*a4*t(i)^2 + 20*a5*t(i)^3;
    end

end