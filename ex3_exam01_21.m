clc;
clear all;

addpath('.\Dynamics\');

syms i1xx i1yy i1zz i1xy i1xz i1yz positive
syms i2xx i2yy i2zz i2xy i2xz i2yz positive
syms m1 m2 positive
syms q1 q2 real
syms q1_dot q2_dot real
syms g0 real
syms d1_c1 d2_c2 real
syms l1 real
syms Fv_1 Fv_2 positive
syms q1_ddot q2_ddot real


T1 =  0.5*i1yy*q1_dot^2;


w2 = [q1_dot * sin(q2); q1_dot * cos(q2); q2_dot];

I2 = [i2xx, 0, 0;
      0, i2yy, 0;
      0, 0, i2zz];

T2rot =  0.5*w2'*I2*w2 + m2*(d2_c2^2)*q1_dot^2;

p_c2 = [d2_c2*sin(q1)*cos(q2); l1 + d2_c2 * sin(q2); d2_c2*cos(q1)*cos(q2)];


v_c2 = diff(p_c2, q1) * q1_dot + diff(p_c2, q2) * q2_dot;

disp("v_c2:")
disp(v_c2)

T2trans = 0.5*m2*(v_c2')*v_c2;
T2 = T2rot + T2trans;
T = T1 + T2;

disp("T:")
disp(T)

M = inertia_matrix_from_kinetic_energy(T, [q1_dot, q2_dot]);
disp("M:")
disp(M)

C = compute_christoffel_matrix(M, [q1, q2], [q1_dot, q2_dot], 2);
disp("C:")
disp(C)

%g = [0; -g0; 0];
U = g0 * d2_c2 * m2 * sin(q2);

g = g_from_potential_energy(U, [q1; q2]);
disp("g:");
disp(g);

Fv = diag([Fv_1, Fv_2]);

Visc = Fv * [q1_dot; q2_dot];
disp("Visc:");
disp(Visc);

Model = M * [q1_ddot; q2_ddot] + C + g + Visc;
disp("Model:");
disp(simplify(Model));