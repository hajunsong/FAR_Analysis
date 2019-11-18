function read_data

    global start_time h g end_time body num_body

    start_time = 0;
    end_time = 2.5;
    h = 0.001;
    g = -9.80665;
    num_body = 6;

    % DH paramter
    % | Link | alpha(deg) |  a(mm)  |  d(mm)   | theta(deg) |
    % |=====================================================|
    % |  1   |    -90     |  0      |   0      |     90     |
    % |  2   |     0      |  151.75 |   0      |    -90     |
    % |  3   |     0      |  150    |   0      |     0      |
    % |  4   |     90     |  86.75  |   0      |     90     |
    % |  5   |     90     |  0      |   -20.25 |     90     |
    % |  6   |     0      |  0      |   102.5  |     0      |

    DH = [-90   0              0         90
            0   0.15175        0        -90
            0   0.150          0          0
           90   0.08675        0         90
           90   0       -0.02025         90
            0   0         0.1025         0];

    % body 0
    body(1).Ai = eye(3);
    body(1).ri = zeros(3,1);
    body(1).Cij = ang2mat(0, 0, 0);
    body(1).sijp = zeros(3,1);
    body(1).ri_dot = [0;0;0];
    body(1).wi = [0;0;0];
    body(1).u_vec = [0;0;1];

    % body 1
    body(2).Cij = ang2mat(DH(1,4), DH(1,1), 0, 1);
    body(2).sijp = [0;0;0];
    body(2).Cii = ang2mat(-pi/2, pi/2, 0);
    body(2).rhoip = [-0.00189888;-1.44683e-8;-0.0234351];
    body(2).mi = 6.33612131907843e-002;
    body(2).Ixx = 5.46760988093101e-005; 
    body(2).Iyy = 4.11897872591055e-005;
    body(2).Izz = 2.28294446378339e-005;
    body(2).Ixy = 1.16933891602143e-011;
    body(2).Iyz = 1.72355337398552e-006;
    body(2).Izx = 3.03099214889948e-011;
    body(2).Jip = [body(2).Ixx body(2).Ixy body(2).Izx;
                    body(2).Ixy body(2).Iyy body(2).Iyz;
                    body(2).Izx body(2).Iyz body(2).Izz];
    body(2).u_vec = [0;0;1];

    % body 2
    body(3).Cij = ang2mat(DH(2,4), DH(2,1), 0, 1);
    body(3).sijp = [0;-DH(2,2);0];
    body(3).Cii = ang2mat(pi, 0, 0);
    body(3).rhoip = [-0.000462227;-0.0427355;0.000759913];
    body(3).mi = 0.291144481135948;
    body(3).Ixx = 6.84357146533933e-004; 
    body(3).Iyy = 1.19767708650701e-004;
    body(3).Izz = 6.2201207394514e-004;
    body(3).Ixy = -7.26891485430593e-006;
    body(3).Iyz = 5.05996626479478e-006;
    body(3).Izx = 1.80750423403909e-007;
    body(3).Jip = [body(3).Ixx body(3).Ixy body(3).Izx;
                    body(3).Ixy body(3).Iyy body(3).Iyz;
                    body(3).Izx body(3).Iyz body(3).Izz];
    body(3).u_vec = [0;0;1];

    % body 3
    body(4).Cij = ang2mat(DH(3,4), DH(3,1), 0, 1);
    body(4).sijp = [DH(3,2);0;0];
    body(4).Cii = ang2mat(-pi/2, 0, 0);
    body(4).rhoip = [0.075;0;0.000807364];
    body(4).mi = 0.416638668104345;
    body(4).Ixx = 1.45776042402133e-003; 
    body(4).Iyy = 1.15949266176089e-004;
    body(4).Izz = 1.44207442743259e-003;
    body(4).Ixy = -2.14630188922107e-014;
    body(4).Iyz = -4.86620428197596e-019;
    body(4).Izx = -5.85663447574856e-020;
    body(4).Jip = [body(4).Ixx body(4).Ixy body(4).Izx;
                    body(4).Ixy body(4).Iyy body(4).Iyz;
                    body(4).Izx body(4).Iyz body(4).Izz];
    body(4).u_vec = [0;0;1];

    % body 4
    body(5).Cij = ang2mat(DH(4,4), DH(4,1), 0, 1);
    body(5).sijp = [0;DH(4,2);0];
    body(5).Cii = ang2mat(-pi/2, 0, 0);
    body(5).rhoip = [0.000749752;0.0609445;0.000415268];
    body(5).mi = 0.228993914748238;
    body(5).Ixx = 7.74704754240776e-005; 
    body(5).Iyy = 2.01161940464821e-004;
    body(5).Izz = 1.88922599914013e-004;
    body(5).Ixy = 2.91568180110741e-006;
    body(5).Iyz = 7.12741866241557e-008;
    body(5).Izx = -2.20466982640091e-006;
    body(5).Jip = [body(5).Ixx body(5).Ixy body(5).Izx;
                    body(5).Ixy body(5).Iyy body(5).Iyz;
                    body(5).Izx body(5).Iyz body(5).Izz];
    body(5).u_vec = [0;0;1];

    % body 5
    body(6).Cij = ang2mat(DH(5,4), DH(5,1), 0, 1);
    body(6).sijp = [0;0;DH(5,3)];
    body(6).Cii = ang2mat(pi, pi/2, 0);
    body(6).rhoip = [0.0555687;0;-0.000237633];
    body(6).mi = 0.204137411295743;
    body(6).Ixx = 9.4696526893192e-005; 
    body(6).Iyy = 7.92107777080459e-005;
    body(6).Izz = 1.38821213983018e-004;
    body(6).Ixy = -2.02238967554624e-005;
    body(6).Iyz = -1.20999283701959e-015;
    body(6).Izx = 1.87131808263915e-015;
    body(6).Jip = [body(6).Ixx body(6).Ixy body(6).Izx;
                    body(6).Ixy body(6).Iyy body(6).Iyz;
                    body(6).Izx body(6).Iyz body(6).Izz];
    body(6).u_vec = [0;0;1];

    % body 6
    body(7).Cij = ang2mat(DH(6,4), DH(6,1), 0, 1);
    body(7).sijp = [0;0;DH(6,3)];
    body(7).Cii = ang2mat(pi/2, pi/2, -pi/2);
    body(7).rhoip = [5.39394e-10;3.33671e-8;0.089384];
    body(7).mi = 2.08363885223627e-002;
    body(7).Ixx = 2.66302463617021e-006; 
    body(7).Iyy = 1.56637607668211e-006;
    body(7).Izz = 1.88187616526518e-006;
    body(7).Ixy = 2.4095425326714e-012;
    body(7).Iyz = 2.738802635816e-013;
    body(7).Izx = 9.27461478843821e-014;
    body(7).Jip = [body(7).Ixx body(7).Ixy body(7).Izx;
                    body(7).Ixy body(7).Iyy body(7).Iyz;
                    body(7).Izx body(7).Iyz body(7).Izz];
    body(7).u_vec = [0;0;1];


end