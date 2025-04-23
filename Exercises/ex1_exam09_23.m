clear all;
clc;

digits(4);

addpath("../Dynamics/");
addpath("../Redundancy/");
addpath("../Robotics1/");
syms q1 q2 q3 q4 real;
syms q1_dot q2_dot q3_dot q4_dot real;
syms q1_ddot q2_ddot q3_ddot q4_ddot real;

q = [q1; q2; q3; q4];
q_dot = [q1_dot; q2_dot; q3_dot; q4_dot];
q_ddot = [q1_ddot; q2_ddot; q3_ddot; q4_ddot];

syms m1 m2 m3 m4 g0 positive;



p_c1 = [q1 / 2; 0];
p_c2 = [q1 / 2; q2 / 2];
p_c3 = [q1 + q3/2; q2];
p_c4 = [q1 + q3; q2 + q4/2];
p_ee = [q1 + q3; q2 + q4];
J = jacobian(p_ee, q);

% Calculate velocities
v_c1 = jacobian(p_c1, q) * q_dot;
v_c2 = jacobian(p_c2, q) * q_dot;
v_c3 = jacobian(p_c3, q) * q_dot; 
v_c4 = jacobian(p_c4, q) * q_dot;

T1 = 0.5 * m1 * (v_c1' * v_c1);
T2 = 0.5 * m2 * (v_c2' * v_c2);
T3 = 0.5 * m3 * (v_c3' * v_c3);
T4 = 0.5 * m4 * (v_c4' * v_c4);

T = T1 + T2 + T3 + T4;
U = m1 * g0 * p_c1(2) + m2 * g0 * p_c2(2) + m3 * g0 * p_c3(2) + m4 * g0 * p_c4(2);

[M, c, S, g] = compute_dynamic_model(q, q_dot, T, U, false);

syms f_x f_y real;

F = [f_x; f_y];

model = M*q_ddot + S*q_dot + g - J' * F;
model = simplify(model);
disp("Model: ");
disp(model);

dJ_dq1 = diff(J, q1);
dJ_dq2 = diff(J, q2);
dJ_dq3 = diff(J, q3);
dJ_dq4 = diff(J, q4);
J_dot = dJ_dq1 * q1_dot + dJ_dq2 * q2_dot + dJ_dq3 * q3_dot + dJ_dq4 * q4_dot;
J_dot = simplify(J_dot);

syms u1 u2 u3 u4 real;
u_q = [u1; u2; u3; u4];

[M_p, c_p, g_p, u_p] = coordinate_transformation(M, c, g, u_q, q_dot, J, J_dot, true);

M_p_curr = subs(M_p, q, [0; 0; 0; 0]);
M_p_curr = subs(M_p_curr, [m1; m2; m3; m4], [1; 1; 1; 1]);
M_p_curr = simplify(M_p_curr);
disp("M_p_curr: ");
disp(M_p_curr);

disp("Daje Roma! f=0, g(q)=0")
J_inv = pinv(J);
M_inv = pinv(M);
disp(M)
u_ns = M*(eye(4) - J_inv * J) * u_q; %proj in the dynamic null space
disp("u_ns: ");
disp(u_ns);

r_curr = [2; -3];
q_curr = [0; 0; 0; 0];
q_dot_curr = [0; 0; 0; 0];
M_inv_curr = subs(M_inv, q, q_curr);
M_inv_curr = subs(M_inv_curr, [m1; m2; m3; m4], [1; 1; 1; 1]);
J_curr = subs(J, q, q_curr);
c_curr = subs(c, [q; q_dot], [q_curr; q_dot_curr]);
u_mn = pinv(J_curr * M_inv_curr) * (r_curr) + c_curr;
disp("u_mn: ");
disp(u_mn);
