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

syms m1 m2 m3 g0 l3 I1 I2 I3 positive;

p_c1 = [0; 0];
p_c2 = [q2 * cos(q1); q2 * sin(q1)] / 2;
p_c3 = [q2 * cos(q1) + l3/2 * cos(q1 + q3); q2 * sin(q1) + l3/2 * sin(q1 + q3)];

v_c2 = jacobian(p_c2, q) * q_dot;
v_c3 = jacobian(p_c3, q) * q_dot;

w1 = q1_dot;
w2 = q1_dot + q3_dot;

T2 = 0.5 * m2 * (v_c2' * v_c2) +  0.5 * w1' * I1 * w1;
T3 = 0.5 * m3 * (v_c3' * v_c3) +  0.5 * w2' * I2 * w2;
T = T2 + T3;
U = m2 * g0 * p_c2(2) + m3 * g0 * p_c3(2);
[M, c, S, g] = compute_dynamic_model(q, q_dot, T, U, false);
g_b = g;
g_b = subs(g_b, [q1; q2; q3], [pi; 0; pi/2]);
disp(g_b); %we balance ee in [0 or pi; 0; +-pi/2] 

model = M*q_ddot + S*q_dot + g;
disp("model: ");
disp(vpa(simplify(model, 3)));
