clc;

clear all;
addpath('.\Dynamics\');

syms i1xx i1yy i1zz i1xy i1xz i1yz positive
syms i2xx i2yy i2zz real
syms rc1x rc1y rc1z real
syms rc2x real
syms l1 l2 real
syms g0 real
syms m1 m2 positive
syms q1 q2 real
syms q1_dot q2_dot real

rc1 = [rc1x; rc1y; rc1z];
rc2 = [rc2x; 0; 0];

Ic1 = [i1xx, i1xy, i1xz;
       i1xy, i1yy, i1yz;
       i1xz, i1yz, i1zz];

Ic2 = diag([i2xx, i2yy, i2zz]);

% dhtable of 2R planar robot

dhtable = [-pi/2, 0, l1, q1;
           0, l2, 0, q2];


T = compute_kinetic_energy("RR", dhtable, [q1, q2], [q1_dot, q2_dot], {rc1, rc2}, [m1, m2], {Ic1, Ic2}, [0; 0; 0], [0; 0; 0], false);

disp("T:")
disp(T)

M = inertia_matrix_from_kinetic_energy(T, [q1_dot, q2_dot]);
C = compute_christoffel_matrix(M, [q1, q2], [q1_dot, q2_dot], 2);


%transform coordinates of center of mass from link frame to base frame
[F, A] = DHMatrix(dhtable);

rc1 = A{1} * [rc1; 1];
rc2 = A{1} * A{2} * [rc2; 1];
rc1 = rc1(1:3);
rc2 = rc2(1:3);

[U, s] = compute_potential_energy_matrix([m1, m2], [q1, q2], {rc1, rc2}, [g0; 0; 0], 2);

disp("U:")
disp(U)

[M, C, S, g] = compute_dynamic_model([q1; q2], [q1_dot; q2_dot], T, U, true);

%g = g_from_potential_energy(U, [q1; q2]);

%disp("M:")
%disp(M)

%disp("C:")
%disp(C)

%disp("g:")
%disp(g)

%S = factorization_S_from_inertia_matrix(M, [q1; q2], [q1_dot; q2_dot], false);
%disp("S:")
%disp(S)
