clc;

clear all;
digits(4);

addpath("./Dynamics/")

% RP planar robot dynamics with kinetic energy

syms q1 q2 real;
syms q1_dot q2_dot real;
syms q1_ddot q2_ddot real;

syms m1 m2 I1 I2 real;
syms d_c1 l1 real;
syms g0 real;

p2 = [l1*cos(q1) + q2 *sin(q1);
      l1*sin(q1) - q2* cos(q1)];

v2 = [diff(p2(1), q1)*q1_dot + diff(p2(1), q2)*q2_dot;
      diff(p2(2), q1)*q1_dot + diff(p2(2), q2)*q2_dot];


T_1 = 0.5 * (I1 + m1*d_c1^2) * q1_dot^2;
T_2 = 0.5 * I2 * q1_dot^2 + 0.5*m2*(v2(1)^2 + v2(2)^2);
T = T_1 + T_2;

rc1 = [d_c1 * cos(q1);
       d_c1 * sin(q1);
       0];

rc2 = [p2; 0];

U = compute_potential_energy_matrix([m1; m2], [q1; q2], {rc1, rc2}, [0; g0; 0], 10);

% compute inertia matrix
[M, c, S, g] = compute_dynamic_model([q1; q2], [q1_dot; q2_dot], T, U, false);

% trajectory 

syms a b k t T real

q_traj = [a + b*(1-cos(pi*t/T)); k];

q_traj_dot = diff(q_traj, t); 

p = M * q_traj_dot;

p = simplify(p);

%integrate p over t
p_int = int(p, t);
p_int = simplify(p_int);
disp('integral of p');
disp(p_int);

%h is the intrgral from 0 to T
h = subs(p_int, t, T) - subs(p_int, t, 0);
h = vpa(simplify(h));
disp('h');
disp(h);

