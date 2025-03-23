syms m11 m22 m33 m23 positive
syms c1 c2 c3 positive
syms tau1 tau2 tau3 positive
syms q1 q2 q3 positive
syms q1_dot q2_dot q3_dot positive
syms p1 p2 p3 real

addpath('..\Dynamics\');

p = [p1; p2; p3];

M = [m11, 0, 0;
    0, m22, m23,
    0, m23, m33];

cq = [c1;
      c2;
      c3];

u_q = [tau1; tau2; tau3];

q = [q1; q2; q3]

q_dot = [q1_dot; q2_dot; q3_dot]

% u_p = inv(J') * tau_q;

% 
% tau_q = [tau1; 
%          tau2 + tau3;
% %          tau3]

g = [0; 0; 0];

J = [1, 0, 0;
     0, 1, 0;
     0, 1, 1]

J_dot = [0, 0, 0;
         0, 0, 0;
         0, 0, 0];


[M_p, c_p, g_p, u_p] = coordinate_transformation(M, cq, g, u_q, q_dot, J, J_dot);

disp('U_p:')
disp(u_p)

disp('M_p:')
disp(M_p)

disp('c_p:')
disp(c_p)