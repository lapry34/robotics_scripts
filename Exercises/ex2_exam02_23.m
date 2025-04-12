clear all;
clc;

digits(4);

addpath("../Dynamics/");
addpath("../Redundancy/");

syms g0 d m Ic positive;
syms q q_dot q_ddot real;

p_c = [
    d * cos(q);
    d * sin(q);
];

v_c = diff(p_c,q)*q_dot;

T = 0.5 * m * (v_c' * v_c) + 0.5 * q_dot * Ic * q_dot;

U = - m *g0 * d*cos(q);
L = T - U;
model = diff(diff(L, q_dot), q_dot)*q_ddot - diff(L, q);
simplify(model)