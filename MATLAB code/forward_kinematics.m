%% -------------------------------------------- theta setting-----------------------------------
syms theta1 theta2 theta3 theta4 theta5 theta6

% theta1 = deg2rad(0);
% theta2 = deg2rad(-50);
% theta3 = deg2rad(30);
% theta4 = deg2rad(60);
% theta5 = deg2rad(70);
% theta6 = deg2rad(-180);
%% -------------------------------------------- DH table parameters-----------------------------------
i1 = [0 0 theta1 135];
i2 = [deg2rad(-90) 0 theta2+deg2rad(-90) 0];
i3 = [0 135 theta3 0];
i4 = [deg2rad(-90) 38 theta4 120];
i5 = [deg2rad(90) 0 theta5 0];
i6 = [deg2rad(-90) 0 theta6+deg2rad(180) 70]; 

%% -------------------------------------------- DH to transformation matrix-----------------------------------
syms alpha a theta d;

T_alpha = [1 0 0 0;0 cos(alpha) -sin(alpha) 0;0 sin(alpha) cos(alpha) 0; 0 0 0 1];
T_a = [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1];
T_theta = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1];
T_d = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];

T = @(alpha, a, theta, d)   [1 0 0 0;0 cos(alpha) -sin(alpha) 0;0 sin(alpha) cos(alpha) 0; 0 0 0 1]...      % T = T_alpha * T_a * T_theta * T_d
                            * [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1]...                                       % functionlize in order to set values easily
                            * [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1]...
                            * [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];                                         
%%  -------------------------------------- DH table to each transformation matrix (Homogeneous matrix and rotation matrix)-------------------------------------------- 
T0_1 = T(i1(1), i1(2), i1(3), i1(4));
T1_2 = T(i2(1), i2(2), i2(3), i2(4));
T2_3 = T(i3(1), i3(2), i3(3), i3(4));
T3_4 = T(i4(1), i4(2), i4(3), i4(4));
T4_5 = T(i5(1), i5(2), i5(3), i5(4));
T5_6 = T(i6(1), i6(2), i6(3), i6(4));

R0_1 = T0_1(1:3, 1:3);
R1_2 = T1_2(1:3, 1:3);
R2_3 = T2_3(1:3, 1:3);
R3_4 = T3_4(1:3, 1:3);
R4_5 = T4_5(1:3, 1:3);
R5_6 = T5_6(1:3, 1:3);

R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6;
T0_6 = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6;          % 6 -> 0
R6_0 = R0_6.';

xyz0 = [T0_6(1,4); T0_6(2,4); T0_6(3,4)];
xyz6 = -R6_0*xyz0;

T6_0 = [R6_0 -R6_0*xyz0;0 0 0 1]

%% ---------------------------------------Transformation between frame{6} and frame{H} -------------------------------
% Note that I neglect the assembly error cased by sensor installation.
% That's means frame{6} = frame{S}

syms x y z raw pitch yaw  %values of setTCP

Rx_raw = [1 0 0; 0 cos(raw) -sin(raw); 0 sin(raw) cos(raw)];
Ry_pitch = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
Rz_yaw = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
R6_H = Rx_raw*Ry_pitch*Rz_yaw;
RH_6 = R6_H.';
TH_6 = [RH_6 -RH_6*[x; y; z];0 0 0 1];

RH_0 = RH_6 * R6_0;
TH_0 = TH_6*T6_0;
%%  --------------------------------------Jacobian matrix -------------------------------------------- 
theta_xyz0 = R0_1*[0; 0 ; theta1] +...              % Based on {0}
             R0_1*R1_2*[0; 0 ; theta2] +...
             R0_1*R1_2*R2_3*[0; 0 ; theta3] +...
             R0_1*R1_2*R2_3*R3_4*[0; 0 ; theta4] +...
             R0_1*R1_2*R2_3*R3_4*R4_5*[0; 0 ; theta5] +...
             R0_1*R1_2*R2_3*R3_4*R4_5*R5_6*[0; 0 ; theta6];
theta_xyz6 = -R6_0*theta_xyz0;

X0 = [xyz0; theta_xyz0];                              % Generalized coordinate
X6 = [xyz6; theta_xyz6];

Jv0 = jacobian(xyz0,[theta1 theta2 theta3 theta4 theta5 theta6]);
%Jw0 = jacobian(theta_xyz0,[theta1 theta2 theta3 theta4 theta5 theta6])
Jw0 = [R0_1(:,3) R0_1*R1_2(:,3) R0_1*R1_2*R2_3(:,3) R0_1*R1_2*R2_3*R3_4(:,3) R0_1*R1_2*R2_3*R3_4*R4_5(:,3) R0_1*R1_2*R2_3*R3_4*R4_5*R5_6(:,3)]
Jg0 = simplify([Jv0;Jw0])   % This jacobian matrix is J0_6 (6 -> 0), see in {0}

%R_Jg0toJg6 = [R6_0 zeros(3,3); zeros(3,3) R6_0];
%Jg6 = simplify(R_Jg0toJg6*Jg0)                                   % This jacobian matrix is J6_0 (0 -> 6), see in {6}

R_Jg0toJgH = [RH_0 zeros(3,3); zeros(3,3) RH_0];
JgH = R_Jg0toJgH*Jg0                                   % This jacobian matrix is JH_0 (0 -> H), see in {H}
%% tricky problem 

% Jv6 = jacobian(xyz6,[theta1 theta2 theta3 theta4 theta5 theta6]);
% Jw6 = jacobian(theta_xyz6,[theta1 theta2 theta3 theta4 theta5 theta6]);
% Jg62 = simplify(jacobian(X6,[theta1 theta2 theta3 theta4 theta5 theta6]));   % This jacobian matrix is J6_0 (0 -> 6), see in {6}
% simplify(Jg62-Jg6)

%I have no idea why the result is incorrect (Jg62 doesn't equare to Jg6). 

%% geometric to anlatical Jacobian
syms alpha beta gamma;
theta = [ [1        0               sin(beta)]              *[alpha beta gamma].' ;
          [0        cos(alpha)      -sin(alpha)*cos(beta)]  *[alpha beta gamma].';
          [0        sin(alpha)      cos(alpha)*cos(beta)]   *[alpha beta gamma].' ];
%Jwe = jacobian(theta,[alpha beta gamma])
Jwe = [1        0               sin(beta);
       0        cos(alpha)      -sin(alpha)*cos(beta);
       0        sin(alpha)      cos(alpha)*cos(beta);]