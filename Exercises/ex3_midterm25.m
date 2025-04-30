clear all;
clc;

digits(4);

addpath("../Dynamics/");
addpath("../Redundancy/");
addpath("../Robotics1/");

%3R spatial robot
syms q1 q2 q3 real;
syms q1_dot q2_dot q3_dot real;
syms q1_ddot q2_ddot q3_ddot real;

q = [q1; q2; q3];
q_dot = [q1_dot; q2_dot; q3_dot];
q_ddot = [q1_ddot; q2_ddot; q3_ddot];
syms g0 positive;
%syms l1 l2 l3 m1 m2 m3 d_c1 d_c2 d_c3 positive;
%syms Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 positive;

l1 = 0.5;
l2 = 0.5;
l3 = 0.5;

d_c1 = l1 * 0.5;
d_c2 = l2 * 0.5;
d_c3 = l3 * 0.5;

m1 = 2;
m2 = 2;
m3 = 2;

Ixx1 = m1/40;
Ixx2 = m2/40;
Ixx3 = m3/40;

Iyy1 = m1/40;
Iyy2 = m2*(0.05 + l2^2)/12;
Iyy3 = m3*(0.05 + l3^2)/12;
syms Izz1 positive;
Izz2 = Iyy2;
Izz3 = Iyy3;

I_c1 = diag([Ixx1, Iyy1, Izz1]);
I_c2 = diag([Ixx2, Iyy2, Izz2]);
I_c3 = diag([Ixx3, Iyy3, Izz3]);

r_c1 = [
    0;
    -l1 + d_c1;
    0;
];

r_c2 = [
    -l2 + d_c2;
    0;
    0;
];

r_c3 = [
    -l3 + d_c3;
    0;
    0;
];

%alpha a d theta
DHTable = [
    pi/2, 0 l1, q1;
    0, l2, 0, q2;
    0, l3, 0, q3;
];

joint_str = "RRR";

cell_r = {r_c1, r_c2, r_c3};
cell_I = {I_c1, I_c2, I_c3};
array_m = [m1; m2; m3];

A = build_transformation_matrices(DHTable);

r_0c1 = A{1} * [r_c1; 1];
r_0c2 = A{1} * A{2} * [r_c2; 1];
r_0c3 = A{1} * A{2} * A{3} * [r_c3; 1];

r_0c1 = r_0c1(1:3);
r_0c2 = r_0c2(1:3);
r_0c3 = r_0c3(1:3);

T0N = A{1} * A{2} * A{3};

cell_r0 = {r_0c1, r_0c2, r_0c3};

T = compute_kinetic_energy(joint_str, DHTable, q, q_dot, cell_r, array_m, cell_I, 0, 0, true);
%[M, c, S, g] = compute_dynamic_model(q, q_dot, T, U, true);
M = inertia_matrix_from_kinetic_energy(T, q_dot);
p_ee = T0N(1:2, 4);

J = jacobian(p_ee, q);
J_dot = compute_J_dot(J, q, q_dot);
q_curr = [3*pi/2; pi/4; 0];
dq_curr = [0; 0; 0];
J_curr = subs(J, q, q_curr);
J_dot_curr = subs(J_dot, [q; q_dot], [q_curr; dq_curr]);
clc;

r_ddot = [-1; -1];
M_curr = subs(M, q, q_curr);

M_2inv = M_curr^(-2);

q_ddot_command = M_2inv * J_curr' * inv(J_curr * M_2inv * J_curr') * (r_ddot - J_dot_curr * dq_curr);
disp(vpa(q_ddot_command));

