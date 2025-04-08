clc;
clear all;
digits(4);

addpath("../Redundancy/")

syms q1 q2 q3 q4 real 
syms dq1 dq2 dq3 dq4 real
q = [q1; q2; q3; q4];
dq = [dq1; dq2; dq3; dq4];

p_ee = [
    cos(q1) + cos(q1 + q2) + cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4);
    sin(q1) + sin(q1 + q2) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4);
];
J = jacobian(p_ee, q);
J_dot = jacobian(p_ee, q) * dq;

% q1 is between -pi/2 and pi/2, q2 is between 0 and pi/2, q3 and q4 are -pi/4 and pi/4

%q0 is the vector of the middle range of each joint
q0 = [0; pi/4; 0; 0];

H_q = (q - q0)' * eye(4,4) * (q - q0);
grad_H = 2 * (q - q0);

r_dot = [-1; 1];
q_curr = [0; pi/2; 0; -pi/4];
J_curr = subs(J, q, q_curr);
grad_H_curr = subs(grad_H, q, q_curr);

q_dot = projected_gradient(J_curr, r_dot, grad_H_curr);

disp("projected gradient q_dot:");
disp(vpa(q_dot));