clear all;
clc;

digits(4);

addpath("../Dynamics/");
addpath("../Redundancy/");

syms q1 q2 real;
syms q1_dot q2_dot real;
syms q1_ddot q2_ddot real;

syms m1 m2 I2 d_c2 l2 g0 positive;

q = [q1; q2];
q_dot = [q1_dot; q2_dot];
q_ddot = [q1_ddot; q2_ddot];

p_c1 = [
    q1;
    0;
];

p_c2 = [
    q1 + d_c2*cos(q2);
    d_c2*sin(q2);
];

p_ee = [
    q1 + l2*cos(q2);
    l2*sin(q2);
];

v_c1 = jacobian(p_c1, q) * q_dot;
v_c2 = jacobian(p_c2, q) * q_dot;
v_ee = jacobian(p_ee, q) * q_dot;

w_c2 = [
    0;
    0;
    q2_dot;
];

T1 = 0.5*m1*(v_c1'*v_c1); 
T2 = 0.5*m2*(v_c2'*v_c2) + 0.5*w_c2'*I2*w_c2;

U = -m1*g0*p_c1(1) - m2*g0*p_c2(1);

[M, c, s, G] = compute_dynamic_model(q, q_dot, T1 + T2, U, true);

model = M * q_ddot + c + G;
disp('Dynamic Model:');
disp(model);

syms ro1 ro2 ro3 ro4 ro5 real;

% d_c2*m2 = ro1
% m2*d_c2^2 + I2 = ro2
% m1 + m2 = ro3

model_ro = [
    -ro1*cos(q2)*q2_dot^2 + ro3*q1_ddot - g0*ro3 - ro1*sin(q2)*q2_ddot;
    q2_ddot*ro2 + ro1*g0*sin(q2) - ro1*q1_ddot*sin(q2);
]

ro = [ro1; ro2; ro3];

Y = extract_Y_v3(model_ro, ro);

disp(Y * ro);

