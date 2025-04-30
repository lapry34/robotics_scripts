clear all;
clc;

digits(4);

addpath("../Dynamics/");
addpath("../Redundancy/");
addpath("../Robotics1/");

syms q1 q2 q3 real;
syms q1_dot q2_dot q3_dot real;
syms q1_ddot q2_ddot q3_ddot real;

q = [q1; q2; q3];
q_dot = [q1_dot; q2_dot; q3_dot];
q_ddot = [q1_ddot; q2_ddot; q3_ddot];
% PPR spatial robot

syms g0 l1 l2 l3 m1 m2 m3 I3 positive;

T1 = 0.5 * m1 * q1_dot^2;
T2 = 0.5 * m2 * (q1_dot^2 + q2_dot^2);

%alpha a d theta
syms d_c1 d_c2 d_c3 real;
DHTable = [
    -pi/2, l1, q1, 0;
    0, -l2, q2, -pi/2;
    0, d_c3, 0, q3;
];

A = build_transformation_matrices(DHTable);
%[p_vec, z_vec, T0N] = direct_kinematics(A);
T0N = [
    sin(q3),  cos(q3), 0,      l1 + d_c3*sin(q3);
      0,        0, 1,                     q2;
    cos(q3), -sin(q3), 0, -l2 + q1 + d_c3*cos(q3);
      0,        0, 0,                      1;
];

p_c1 = [
    d_c1;
    0;
    q1;
];

p_c2 = [
    l1;
    q2;
    q1 - d_c2;
];

p_c3 = T0N(1:3, 4);


v_c3 = jacobian(p_c3, q) * q_dot;

disp("p_c3");
disp(p_c3);
disp("v_c3");
disp(v_c3);

T3 = 0.5 * m3 * (v_c3' * v_c3) + 0.5*I3*q3_dot^2;

T = T1 + T2 + T3;
U = m1 * g0 * p_c1(3) + m2 * g0 * p_c2(3) + m3 * g0 * p_c3(3);
disp("U");
disp(U);


[M, c, S, g] = compute_dynamic_model(q, q_dot, T, U, true);

M_dot = inertia_matrix_derivative(M, q, q_dot);

S_2 = [
    0, 0, -d_c3*m3*q3_dot*cos(q3);
    q2_dot, -q1_dot, 0;
    0, 0, 0;
];

C_2 = S_2 * q_dot;
disp("C obtained from S_2: ");
disp(C_2);
disp("origianal C: ");
disp(c);

disp("M_dot  2*S_2: ");
disp(M_dot - 2*S_2);

model = M * q_ddot + c + g;
disp("model");
disp(model);

% a1 = d_c3 * m3;
% a2 = m2 + m3;
% a3 = m1;
% a4 = m3*d_c3^2 + I3;

syms a1 a2 a3 a4 real;

model_ro = [
    -a1*cos(q3)*q3_dot^2 + g0*(a2 + a3) + q1_ddot*(a2 + a3) - a1*q3_ddot*sin(q3);
    q2_ddot*a2;
    q3_ddot*a4 - a1*g0*sin(q3) - a1*q1_ddot*sin(q3);
];
disp("model_ro");
disp(model_ro);

a = [a1; a2; a3; a4];

Y = extract_Y_v3(model_ro, a);
disp("Y");
disp(Y);

syms omega t real;

q_d = [0; sin(2*omega*t); omega*t];
q_d_dot = diff(q_d, t)
q_d_ddot = diff(q_d_dot, t)

torque = subs(model, [q; q_dot; q_ddot], [q_d; q_d_dot; q_d_ddot])
