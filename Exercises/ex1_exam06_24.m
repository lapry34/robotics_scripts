clc;
clear all;
digits(4);

addpath("../Dynamics/")

syms q1 q2 q3 a2 a3 real
syms dq1 dq2 dq3 real
syms ddq1 ddq2 ddq3 real

joints_str = "RRR";
% alpha a d theta
DHTable = [
    pi/2 0 0 q1;
    0 a2 0 q2;
    0 a3 0 q3;
];

syms r1_x r1_y r1_z real
syms r2_x r2_y r2_z real
syms r3_x r3_y r3_z real
syms m1 m2 m3 positive
syms i1x i1y i1z i2x i2y i2z i3x i3y i3z real
syms dc1 dc2 dc3 real


r1 = [0; dc1; 0];
r2 = [0; a2-dc2; 0];
r3 = [0; a3-dc3; 0];

r_c = {r1; r2; r3};
I1 = diag([i1x; i1y; i1z]);
I2 = diag([i2x; i2y; i2z]);
I3 = diag([i3x; i3y; i3z]);

T = compute_kinetic_energy(joints_str, DHTable, [q1; q2; q3],...
 [dq1; dq2; dq3], r_c, [m1; m2; m3], [I1; I2; I3], 0, 0, 0);

T = simplify(T);
M = compute_inertia_matrix(T, [dq1; dq2; dq3], 3);

disp("M:")
disp(M)

syms ro1 ro2 ro3 ro4 ro5 ro6 real

M_ro = [
    ro1 + ro2 * cos(q2)^2 + ro3 * cos(q2 + q3)^2 + 2 * ro5 * cos(q2) * cos(q2+q3), 0, 0;
    0, ro4 + 2 * ro5 * cos(q3), ro6 + 2 * ro5 * cos(q3);
    0, ro6 + 2 * ro5 * cos(q3), ro6;
];

Y = extract_Y_v3(M_ro * [ddq1; ddq2; ddq3], [ro1; ro2; ro3; ro4; ro5; ro6]);

disp("Y:")
disp(Y)