clear all;
clc;

digits(4);

addpath("../Dynamics/");

%2P planar robot having the two prismatic joint axes 
% skewed by a fixed angle pi/3
% Adding a third link with revolute joint (PPR robot)

syms q1 q2 q3 real;
syms q1_dot q2_dot q3_dot real;
syms q1_ddot q2_ddot q3_ddot real;
q = [q1;q2;q3];
q_dot = [q1_dot;q2_dot;q3_dot];
q_ddot = [q1_ddot;q2_ddot;q3_ddot];

% Parameters for all links
%l3 = 0.5;  % length of third link in meters
%m1 = 1;    % mass of first link in kg (example value)
%m2 = 1;    % mass of second link in kg (example value)
%m3 = 2;    % mass of third link in kg
%I1 = 0;    % inertia of first link (prismatic joint)
%I2 = 0;    % inertia of second link (prismatic joint)
%d = 0.2;    % distance between prismatic joints in meters

syms m1 m2 m3 I1 I2 positive;
syms d l3 real;
I3 = m3*l3^2/12;  % barycentric inertia normal to motion plane for third link

%alpha a d theta
DHTable = [
    pi/3 0 q1 0;
    0 0 q2 0;
    0 l3 0 q3;  % Third link with revolute joint
];

% Define joint types (P = prismatic, R = revolute)
joint_types = "PPR";

% Define center of mass positions relative to respective link frames
r1_c1 = [0; q1/2; 0];  % Center of mass of link 1 in frame 1
r2_c2 = [0; d; 0];  % Center of mass of link 2 in frame 2
r3_c3 = [l3/2; 0; 0];  % Center of mass of link 3 in frame 3

% Cell array of center of mass positions
cell_ri_ci = {r1_c1, r2_c2, r3_c3};

% Array of masses
array_m_i = [m1, m2, m3];

% Cell array of inertia tensors (for planar robot, only z-component matters)
cell_I_ci = [I1, I2, I3];

% Initial velocity and angular velocity of the base
v0_0 = [0; 0; 0];
omega0_0 = [0; 0; 0];
syms g0 positive;
% Compute kinetic energy using the provided function
T = compute_kinetic_energy(joint_types, DHTable, q, q_dot, cell_ri_ci, array_m_i, cell_I_ci, v0_0, omega0_0, false);
% Compute inertia matrix using the provided function

[boh, A] = DHMatrix(DHTable);

r1_c0 = A{1}(1:3, 1:3) * r1_c1;
r2_c0 = A{1}(1:3, 1:3) * A{2}(1:3, 1:3) * r2_c2;
r3_c0 = A{1}(1:3, 1:3) * A{2}(1:3, 1:3) * A{3}(1:3, 1:3) * r3_c3;

cell_ri_c0 = {r1_c0, r2_c0, r3_c0};

U = compute_potential_energy_matrix([m1 m2 m3], q, cell_ri_c0, [0; -g0; 0], 3);

[M, c, S, G] = compute_dynamic_model(q, q_dot, T, U, true);


%display dynamic model M*q_ddot + c + G 
disp("Dynamic Model:");
disp("M*q_ddot + c + G =");
disp(vpa(simplify(M*q_ddot + c + G)));

%m1 + m2 + m3, m1, m3, m2 +m3
syms ro1 ro2 ro3 ro4 real;
model = [
    -1.3*l3*ro3*sin(q3)*q3_dot^2 - 0.25*g0*ro2 + 0.5*q2_ddot*ro4 + ro1*q1_ddot + 1.3*l3*ro3*cos(q3)*q3_ddot;
    0.5*ro4*(q1_ddot + 2*q2_ddot);
    0.083*l3*ro3*(28*l3*q3_ddot - 3*g0*cos(q3) + 15.59*q1_ddot*cos(q3));
];

disp("Model:");
disp(vpa(model));

Y = extract_Y_v3(model, [ro1 ro2 ro3 ro4]);

disp("Y:");
disp(vpa(Y));

%check
disp("ckeck:")
disp(vpa(Y*[ro1;ro2;ro3;ro4]));