clc;

clear all;
digits(4);

addpath("./Dynamics/")

syms q1 q2 q3 real;
syms q1_dot q2_dot q3_dot real;
syms q1_ddot q2_ddot q3_ddot real;

syms m1 m2 m3 positive;
syms d_c1 d_c2 d_c3 l1 l2 l3 real;
syms g0 real;
syms i1xx i1yy i1zz positive
syms i2xx i2yy i2zz positive
syms i3xx i3yy i3zz positive
syms rc1x rc1y rc1z real
syms rc2x rc2y rc2z real
syms rc3x rc3y rc3z real

I1 = 0; %diag([i1xx, i1yy, i1zz]);
I2 = diag([0, i2yy, 0]);%diag([i2xx, i2yy, i2zz]);
I3 = diag([i3xx, i3yy, i3zz]);
r_c1 = [0; 0; rc1z];
r_c2 = [0; 0; d_c2];
r_c3 = [0; d_c3; 0];

r_c = {r_c1, r_c2, r_c3};

% DHTable of PRR 3D robot
% alpha a d theta
DHTable = [-pi/2, 0, q1, 0;
          pi/2  0,  l2, q2;
            0,    d_c3, 0, q3];

% Compute the kinetic energy of the system
T = compute_kinetic_energy("PRR", DHTable, [q1; q2; q3], [q1_dot; q2_dot; q3_dot], r_c, [m1; m2; m3], {I1, I2, I3}, [0; 0; 0], [0; 0; 0], false);

T1 = 0.5 * m1 * q1_dot^2;
T2 = 0.5 * m2 * q1_dot^2 + 0.5 * i2yy * q2_dot^2;

[F, A] = DHMatrix(DHTable);

R_ee = F(1:3, 1:3);
J = PartialGeometricJacobian(DHTable, 'PRR');

J_a = J(1:3, :);
J_w = J(4:6, :);

v_ee = J_a * [q1_dot; q2_dot; q3_dot];
w_ee = J_w * [q1_dot; q2_dot; q3_dot];

T3 = 0.5 * m3 * (v_ee' * v_ee) + 0.5 * (w_ee' * I3 * w_ee);
%T = T1 + T2 + T3;
T = simplify(T);


% Compute the potential energy of the system
U = compute_potential_energy_matrix([m1; m2; m3], [q1; q2; q3], r_c, [0; 0; g0], 10);

% Compute the model of the system
[M, c, S, g] = compute_dynamic_model([q1; q2; q3], [q1_dot; q2_dot; q3_dot], T, U, true);
D = inertia_matrix_partial_jacobians(DHTable, 'RRP', [m1; m2; m3], {I1, I2, I3});
D = simplify(D);
disp(D);