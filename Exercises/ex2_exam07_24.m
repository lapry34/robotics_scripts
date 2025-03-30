clc;

clear all;
digits(4);

addpath("./Redundancy/")

syms q1 q2 real;
syms q1_dot q2_dot real;
syms q1_ddot q2_ddot real;
syms l k1 k2 real;


Kv = diag([k1, k2]);


q0_ddot = - Kv * [q1_dot; q2_dot];

p = [l*cos(q1) + l*cos(q1 + q2)];

J = jacobian(p, [q1; q2]);
dJ_dq1 = diff(J, q1);
dJ_dq2 = diff(J, q2);
J_dot = dJ_dq1 * q1_dot + dJ_dq2 * q2_dot;
J_dot = simplify(J_dot);

syms r_ddot real;



q_ddot_command = projected_gradient_dynamics(J, J_dot, r_ddot, [q1_dot; q2_dot], q0_ddot, 0);
q_ddot_command = simplify(q_ddot_command);
disp("commanded joint acceleration")

q_ddot_command = subs(q_ddot_command, [q1; q2; q1_dot; q2_dot], [pi/4; -pi/2; 1; -1]);
q_ddot_command = subs(q_ddot_command, [r_ddot; k1; k2; l], [1; 2; 2; 1]);
q_ddot_command = vpa(q_ddot_command);

disp(q_ddot_command)