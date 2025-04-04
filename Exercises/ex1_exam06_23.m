clc;

clear all;
digits(4);

addpath("./Dynamics/")

syms q1 q2 q3 q4 real;
syms q1_dot q2_dot q3_dot q4_dot real;
syms q1_ddot q2_ddot q3_ddot q4_ddot real;

syms m1 m2 m3 m4 positive;
syms d_c1 d_c2 d_c3 d_c4 l1 l2 l3 l4 real;
syms g0 real;

syms I_c1 I_c2 I_c3 I_c4 positive

%4R robot under gravity, absolute coordinates


T1 = 0.5 * (I_c1 + m1*d_c1^2) * q1_dot^2;

p_c2 = [l1 * cos(q1) + d_c2 * cos(q2);
        l1 * sin(q1) + d_c2 * sin(q2)];

v_c2 = jacobian(p_c2, [q1; q2]) * [q1_dot; q2_dot];


T2 = 0.5 * (I_c2) * q2_dot^2 + 0.5 * m2 * (v_c2' * v_c2);

p_c3 = [l1 * cos(q1) + l2 * cos(q2) + d_c3 * cos(q3);
        l1 * sin(q1) + l2 * sin(q2) + d_c3 * sin(q3)];

v_c3 = jacobian(p_c3, [q1; q2; q3]) * [q1_dot; q2_dot; q3_dot];

T3 = 0.5 * (I_c3) * q3_dot^2 + 0.5 * m3 * (v_c3' * v_c3);

p_c4 = [l1 * cos(q1) + l2 * cos(q2) + l3 * cos(q3) + d_c4 * cos(q4);
        l1 * sin(q1) + l2 * sin(q2) + l3 * sin(q3) + d_c4 * sin(q4)];
v_c4 = jacobian(p_c4, [q1; q2; q3; q4]) * [q1_dot; q2_dot; q3_dot; q4_dot];

T4 = 0.5 * (I_c4) * q4_dot^2 + 0.5 * m4 * (v_c4' * v_c4);
T = T1 + T2 + T3 + T4;
T = simplify(T);

M = inertia_matrix_from_kinetic_energy(T, [q1_dot; q2_dot; q3_dot; q4_dot]);
M = simplify(M);

disp("M = ")
disp(M)

J = [
    1 0 0 0;
    1 1 0 0;
    1 1 1 0;
    1 1 1 1;
];
J_inv = inv(J);

syms theta1 theta2 theta3 theta4 real;
syms theta1_dot theta2_dot theta3_dot theta4_dot real;
syms theta1_ddot theta2_ddot theta3_ddot theta4_ddot real;
M_tmp = subs(M, [q1; q2; q3; q4], T*[theta1; theta2; theta3; theta4]);
M_theta = J' * M_tmp * J;
M_theta = simplify(M_theta);
disp("M_theta = ")
disp(M_theta)
