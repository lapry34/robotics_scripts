clear all;
clc;

digits(4);

addpath("../Dynamics/");
addpath("../Redundancy/");

syms q1 q2 real;
syms q1_dot q2_dot real;
syms q1_ddot q2_ddot real;

syms g0 m1 I1 m2 I2 positive;
syms l1 l2 positive;
syms Fv1 Fv2 real;

Fv = [Fv1; Fv2];

T1 = 0.5*(I1 + m1*(l1/2)^2)*q1_dot^2;


p_c2 = [l1*cos(q1); l1*sin(q1)];
p_c1 = p_c2/2;
v_c2 = diff(p_c2,q1) * q1_dot;
w_2 = [0; 0; q1_dot + q2_dot];

T2 = 0.5*m2*(v_c2'*v_c2) + 0.5*w_2'*I2*w_2;


U = compute_potential_energy_matrix([m1; m2], [q1 ;q2], {p_c1; p_c2}, [g0; 0], 3);

[M, c, s, G] = compute_dynamic_model([q1; q2], [q1_dot; q2_dot], T1 + T2, U, true);

model = M * [q1_ddot; q2_ddot] + c + G + Fv'*[q1_dot; q2_dot];
disp('Dynamic Model:');
disp(model);

syms ro1 ro2 ro3 ro4 ro5 real;

% I2 = ro1
% (I1 + I2 + (l1^2*m1)/4 + l1^2*m2) = ro2
% - 0.5*g0*l1*(m1 + 2.0*m2) ro3
% Fv = ro4;
model_ro = [
    ro1*q2_ddot + q1_ddot*ro2 + ro3*sin(q1) + ro4*q1_dot;
    ro1*q1_ddot + ro1*q2_ddot + ro5*q2_dot;
];

ro = [ro1; ro2; ro3; ro4; ro5];

Y = extract_Y_v4(model_ro, ro, true);

syms delta q2_c q1_fin q1_in real;

Fv = 0;

syms t T real;

%tau = t/T;
syms tau real;
q_in = [q1_in; q2_c];
q_fin = [q1_fin; q2_c];
delta_q = q_fin - q_in;
q_tau = q_in + delta_q* (-2 * tau^3 + 3 * tau^2);
q_tau_dot = diff(q_tau, tau) * 1/T;
q_tau_ddot = diff(q_tau_dot, tau) * 1/T;

M_inv = inv(M);

torque_tau = M * (q_tau_ddot);

% torques(t)
disp("torque(t)");
disp(torque_tau);