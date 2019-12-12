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
    % |  2   |      0     |  164.25 |   16.50  |    -90     |
    % |  3   |    180     |  170    |   0      |      0     |
    % |  4   |     90     |  65.25  |   0      |     90     |
    % |  5   |     90     |  0      |   -20.25 |     90     |
    % |  6   |      0     |  0      |   102.5  |      0     |

    DH = [-90   0              0        -90
            0   0.16425        0.0165   -90
          180   0.170          0          0
           90   0.06525        0         90
           90   0       -0.01675         90
            0   0          0.084         0];

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
    body(2).Cii = ang2mat(0,0,0);
    body(2).rhoip = [-0.000135874;-1.068e-12;-0.0246531];
    body(2).mi = 7.08684533226054e-002;
    body(2).Ixx = 2.15089916203442e-005; 
    body(2).Iyy = 3.65070109080493e-005;
    body(2).Izz = 2.83989018839352e-005;
    body(2).Ixy = -1.24014643551512e-017;
    body(2).Iyz = -1.12222253664962e-010;
    body(2).Izx = 3.33296217281281e-008;
    body(2).Jip = [body(2).Ixx body(2).Ixy body(2).Izx;
                    body(2).Ixy body(2).Iyy body(2).Iyz;
                    body(2).Izx body(2).Iyz body(2).Izz];
    body(2).u_vec = [0;0;1];

    % body 2
    body(3).Cij = ang2mat(DH(2,4), DH(2,1), 0, 1);
    body(3).sijp = [-DH(2,3);-DH(2,2);0];
    body(3).Cii = ang2mat(0,pi/2,-pi/2);
    body(3).rhoip = [0.00264336;-0.0319009;0.000524792];
    body(3).mi = 0.233270004294732;
    body(3).Ixx = 4.08019849963512e-004; 
    body(3).Iyy = 4.36730722674176e-004;
    body(3).Izz = 8.71040040349363e-005;
    body(3).Ixy = -3.23596533711006e-007;
    body(3).Iyz = -4.58645443586337e-005;
    body(3).Izx = -2.40916440855742e-006;
    body(3).Jip = [body(3).Ixx body(3).Ixy body(3).Izx;
                    body(3).Ixy body(3).Iyy body(3).Iyz;
                    body(3).Izx body(3).Iyz body(3).Izz];
    body(3).u_vec = [0;0;1];

    % body 3
    body(4).Cij = ang2mat(DH(3,4), DH(3,1), 0, 1);
    body(4).sijp = [DH(3,2);0;0];
    body(4).Cii = ang2mat(pi/2,pi/2,-pi/2);
    body(4).rhoip = [0.0668431;4.49044e-11;0.000574255];
    body(4).mi = 0.294733648136712;
    body(4).Ixx = 1.33438729955757e-003; 
    body(4).Iyy = 1.35236609017727e-003;
    body(4).Izz = 1.35236609017727e-003;
    body(4).Ixy = -1.65934500194573e-013;
    body(4).Iyz = 5.8944693629749e-013;
    body(4).Izx = -1.46477397988517e-006;
    body(4).Jip = [body(4).Ixx body(4).Ixy body(4).Izx;
                    body(4).Ixy body(4).Iyy body(4).Iyz;
                    body(4).Izx body(4).Iyz body(4).Izz];
    body(4).u_vec = [0;0;1];

    % body 4
    body(5).Cij = ang2mat(DH(4,4), DH(4,1), 0, 1);
    body(5).sijp = [0;DH(4,2);0];
    body(5).Cii = ang2mat(pi/2,pi/2,-pi/2);
    body(5).rhoip = [0.000488263;0.0465912;3.24848e-5];
    body(5).mi = 0.108749563323237;
    body(5).Ixx = 5.19451711277109e-005; 
    body(5).Iyy = 2.17677195227188e-005;
    body(5).Izz = 5.3423100843467e-005;
    body(5).Ixy = -5.50565486810879e-008;
    body(5).Iyz = -4.1508751024039e-007;
    body(5).Izx = -1.72489029231613e-009;
    body(5).Jip = [body(5).Ixx body(5).Ixy body(5).Izx;
                    body(5).Ixy body(5).Iyy body(5).Iyz;
                    body(5).Izx body(5).Iyz body(5).Izz];
    body(5).u_vec = [0;0;1];

    % body 5
    body(6).Cij = ang2mat(DH(5,4), DH(5,1), 0, 1);
    body(6).sijp = [0;0;DH(5,3)];
    body(6).Cii = ang2mat(-pi/2, 0, 0);
    body(6).rhoip = [0.0449512;-1.30501e-12;-0.00250684];
    body(6).mi = 0.110204790536652;
    body(6).Ixx = 5.65872649539517e-005; 
    body(6).Iyy = 3.21370607982722e-005;
    body(6).Izz = 3.83261110287993e-005;
    body(6).Ixy = 3.11849020965568e-015;
    body(6).Iyz = 2.2933899377825e-006;
    body(6).Izx = 1.98601435820104e-015;
    body(6).Jip = [body(6).Ixx body(6).Ixy body(6).Izx;
                    body(6).Ixy body(6).Iyy body(6).Iyz;
                    body(6).Izx body(6).Iyz body(6).Izz];
    body(6).u_vec = [0;0;1];

    % body 6
    body(7).Cij = ang2mat(DH(6,4), DH(6,1), 0, 1);
    body(7).sijp = [0;0;DH(6,3)];
    body(7).Cii = ang2mat(pi, pi/2, 0);
    body(7).rhoip = [4.25231e-8;0.00049999;0.0756101];
    body(7).mi = 1.06900256777816e-002;
    body(7).Ixx = 5.4230201398644e-007; 
    body(7).Iyy = 8.27349038593228e-007;
    body(7).Izz = 6.80541319418483e-007;
    body(7).Ixy = 1.68734917731254e-012;
    body(7).Iyz = 1.25961065157397e-013;
    body(7).Izx = 3.11703037783589e-013;
    body(7).Jip = [body(7).Ixx body(7).Ixy body(7).Izx;
                    body(7).Ixy body(7).Iyy body(7).Iyz;
                    body(7).Izx body(7).Iyz body(7).Izz];
    body(7).u_vec = [0;0;1];


end