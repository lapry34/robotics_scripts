clear all;
clc;

digits(4);

addpath("../Dynamics/");
addpath("../Redundancy/");


syms q1 q2 q3 real;
syms q1_dot q2_dot q3_dot real;
syms q1_ddot q2_ddot q3_ddot real;
syms l m positive;
d = l/2;
Ic = m*l^2/12;

T = [
    1 0 0;
    1 1 0;
    1 1 1;
];

p_c1 = [
    d * cos(q1);
    d * sin(q1);
];

p_c2 = [
    l * cos(q1) + d*cos(q1+q2);
    l * sin(q1) + d*sin(q1+q2);
];

p_c3 = [
    l * cos(q1) + l*cos(q1+q2) + d*cos(q1+q2+q3);
    l * sin(q1) + l*sin(q1+q2) + d*sin(q1+q2+q3);
];

p_ee = [
    l * cos(q1) + l*cos(q1+q2) + l*cos(q1+q2+q3);
    l * sin(q1) + l*sin(q1+q2) + l*sin(q1+q2+q3);
];

v_c1 = jacobian(p_c1, [q1; q2; q3])*[q1_dot; q2_dot; q3_dot];
v_c2 = jacobian(p_c2, [q1; q2; q3])*[q1_dot; q2_dot; q3_dot];
v_c3 = jacobian(p_c3, [q1; q2; q3])*[q1_dot; q2_dot; q3_dot];

w_1 = [0; 0; q1_dot];
w_2 = [0; 0; q1_dot + q2_dot];
w_3 = [0; 0; q1_dot + q2_dot + q3_dot];

T1 = 0.5*m*(v_c1' * v_c1) + 0.5*w_1'*Ic*w_1;
T2 = 0.5*m*(v_c2' * v_c2) + 0.5*w_2'*Ic*w_2;
T3 = 0.5*m*(v_c3' * v_c3) + 0.5*w_3'*Ic*w_3;

M = compute_inertia_matrix(T1 + T2 + T3, [q1_dot; q2_dot; q3_dot], 3);

q_curr = [pi/4; -pi/2; pi/2];
p_ddot_des = [1; 0];

J = jacobian(p_ee, [q1; q2; q3]);
dJ_dq1 = diff(J, q1);
dJ_dq2 = diff(J, q2);
dJ_dq3 = diff(J, q3);
J_dot = dJ_dq1 * q1_dot + dJ_dq2 * q2_dot + dJ_dq3 * q3_dot;
J_dot = simplify(J_dot);

J_curr = subs(J, [q1; q2; q3], q_curr);
J_dot_curr = subs(J_dot, [q1; q2; q3], q_curr);
J_dot_curr = subs(J_dot_curr, [q1_dot; q2_dot; q3_dot], [0; 0; 0]);

M_curr = subs(M, [q1; q2; q3], q_curr);
M_inv = inv(M_curr);
J_pinv = pinv(J_curr);

q_ddot_A = J_pinv * (p_ddot_des);
tau_A = M_curr * q_ddot_A;
disp(vpa(tau_A));

W = T' * T;

J_abs_pinv = weighted_pinv(J_curr, W);
tau_B = M_curr * J_abs_pinv * p_ddot_des;
disp(vpa(tau_B));

J_M_pinv = weighted_pinv(J_curr, M_curr);
tau_C = M_curr * J_M_pinv * p_ddot_des;
disp(vpa(tau_C));

vpa(M_curr)