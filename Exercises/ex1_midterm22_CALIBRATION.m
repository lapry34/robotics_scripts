clear all;
clc;

digits(4);

addpath("../Dynamics/");
addpath("../Redundancy/");
addpath("../Calibration/");
addpath("../Robotics1/");

syms q1 q2 real;
syms q1_dot q2_dot real;
syms q1_ddot q2_ddot real;

syms l1_cap l2_cap l1 l2 real;

q = [q1; q2];
q_dot = [q1_dot; q2_dot];
q_ddot = [q1_ddot; q2_ddot];

% planar 2R robot calibration

DHTable_measured = [
    0, l1_cap, 0, q1;
    0, l2_cap, 0, q2;
];

A = build_transformation_matrices(DHTable_measured);


[p_vec, z_vec, T0N] = direct_kinematics(A);

p_ee_cap = T0N(1:2, 4); %1:2 planar!!!
p_ee = subs(p_ee_cap, l1_cap, l1);
p_ee = subs(p_ee, l2_cap, l2);

sym_delta_p = p_ee - p_ee_cap;
sym_delta_p = simplify(sym_delta_p);
disp("delta_p = [" + join(string(sym_delta_p), "; ") + "];");

p_ee = subs(p_ee, l1, 1);
p_ee = subs(p_ee, l2, 1);

q_a = [0; 0];
q_b = [pi/2; 0];
q_c = [pi/4; -pi/4];
q_d = [0; pi/4];

p_a = [2; 0];
p_b = [0; 2];
p_c = [1.6925; 0.7425];
p_d = [1.7218; 0.6718];

p_a_cap = subs(p_ee, q, q_a);
p_b_cap = subs(p_ee, q, q_b);
p_c_cap = subs(p_ee, q, q_c);
p_d_cap = subs(p_ee, q, q_d);

p = [p_a; p_b; p_c; p_d];
p_cap = [p_a_cap; p_b_cap; p_c_cap; p_d_cap];

delta_p = p - p_cap;

delta_p = simplify(delta_p);

disp("delta_p = [" + join(string(delta_p), "; ") + "];");

phi_q = [cos(q1), cos(q1 + q2); sin(q1), sin(q1 + q2)];

phi_q_a = subs(phi_q, q, q_a);
phi_q_b = subs(phi_q, q, q_b);
phi_q_c = subs(phi_q, q, q_c);
phi_q_d = subs(phi_q, q, q_d);
phi = [phi_q_a; phi_q_b; phi_q_c; phi_q_d];

delta_l = pinv(phi) * delta_p;

disp("delta_l = [" + join(string(vpa(delta_l)), "; ") + "];");