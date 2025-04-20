clear all;
clc;

digits(4);

addpath("../Dynamics/");
addpath("../Redundancy/");

syms q1 q2 real;
syms q1_dot q2_dot real;
syms q1_ddot q2_ddot real;

q = [q1; q2];
q_dot = [q1_dot; q2_dot];
q_ddot = [q1_ddot; q2_ddot];

syms m1 m2 I1 I2 g0 positive;
syms l1 l2 real;

syms dc_2 real;

T1 = 0.5 * I1 * q1_dot^2;

p_c2 = [
    q2 * cos(q1);
    q2 * sin(q1)
];

v_c2 = diff(p_c2, q1) * q1_dot + diff(p_c2, q2) * q2_dot;


T2 = 0.5 * m2 * (v_c2' * v_c2) + 0.5 * I2 * q1_dot^2;
U = m2 * g0 * p_c2(2);

[M, c, S, g] = compute_dynamic_model(q, q_dot, T1 + T2, U, true);

model = M*q_ddot + c + g;

M_dot = inertia_matrix_derivative(M, q, q_dot);

disp("Model:");
disp(simplify(model));

x1 = q;
x2 = M * q_dot;

m1_c = 10;
m2_c = 5;
I1_c = 0.1;
I2_c = 0.5;
q_c = [pi/4; 1];
q_dot_c = [2; -0.5];
tau_c = [1; 0];

p_dot = tau_c - subs(c, [q; q_dot], [q_c; q_dot_c]) - subs(g, q, q_c);
p_dot = p_dot + subs(M_dot, [q; q_dot], [q_c; q_dot_c]) * q_dot_c;

disp("p_dot:");
p_dot = subs(p_dot, [m1; m2; I1; I2; g0], [m1_c; m2_c; I1_c; I2_c; 9.81]);
disp(vpa(p_dot));

