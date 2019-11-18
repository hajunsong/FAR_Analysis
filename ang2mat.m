function mat = ang2mat(varargin)

    ang_z1 = varargin{1};
    ang_x = varargin{2};
    ang_z2 = varargin{3};
    
    if length(varargin) > 3
        if varargin{4} == 1
            z1 = ang_z1*pi/180;
            x =  ang_x*pi/180;
            z2 = ang_z2*pi/180; 
        else
            z1 = ang_z1;
            x =  ang_x;
            z2 = ang_z2;
        end
    else
        z1 = ang_z1;
        x =  ang_x;
        z2 = ang_z2;
    end

    Rz1 = [cos(z1) -sin(z1) 0;sin(z1) cos(z1) 0;0 0 1];
    Rx = [1 0 0;0 cos(x) -sin(x);0 sin(x) cos(x)];
    Rz2 = [cos(z2) -sin(z2) 0;sin(z2) cos(z2) 0;0 0 1];

    mat = Rz1*Rx*Rz2;
    
end