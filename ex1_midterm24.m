clc;

clear all;
digits(4);
addpath('~/Documents/GitHub/robotics_scripts/Redundancy/')
% 3R planar robot direct kinematics function, with unitary links

% Input: joint angles q1, q2, q3
% Output: end-effector position (x, y)
syms q1 q2 q3 real;

p_ee = [cos(q1) + cos(q1 + q2) + cos(q1 + q2 + q3); sin(q1) + sin(q1 + q2) + sin(q1 + q2 + q3)];

% compute jacobian of p_ee
J = jacobian(p_ee, [q1, q2, q3]);

% display jacobian
disp('Jacobian of p_ee:');
disp(J);

v_e = [0; 1];

% q1=0, q2=pi/2, q3=-pi/2
% compute the jacobian
J = subs(J, [q1, q2, q3], [0, pi/2, -pi/2]);


H = norm([cos(q1) + cos(q1 + q2); sin(q1) + sin(q1 + q2)]  - [0 ; 2]) - 0.5;

% compute the gradient of H
grad_H = jacobian(H, [q1, q2, q3]);
q0_dot = grad_H';
I = eye(length(J));
q_dot = simplify(pinv(J) * v_e + (I - pinv(J) * J) * q0_dot);
q_dot = projected_gradient(J, v_e, q0_dot);

q_dot = subs(q_dot, [q1, q2, q3], [0, pi/2, -pi/2]);

disp('q_dot:');
disp(vpa(q_dot));

J2 = jacobian([cos(q1) + cos(q1 + q2); sin(q1) + sin(q1 + q2)], [q1, q2, q3]);
J2 = subs(J2, [q1, q2, q3], [0, pi/2, -pi/2]);
r2_dot = J2 * q_dot;
P1 = (I - pinv(J) * J);
q_dot2 = pinv(J) * v_e +  P1 * pinv(J2 * P1) * (r2_dot - J2 * pinv(J) * v_e);

q_dot3 = task_priority(J, v_e, J2, r2_dot);

disp(vpa(q_dot2));

disp(vpa(q_dot3));

% use reduced gradient
q_list = [q1, q2, q3];
a_list = [1, 2];
b_list = [3];

q_dot4 = reduced_gradient(v_e, q_list, a_list, b_list, J, q0_dot);
q_dot4 = subs(q_dot4, [q1, q2, q3], [0, pi/2, -pi/2]);
disp("reduced gradient")
disp(vpa(q_dot4));