clear variables;
clc

syms l1 l2 q1 q2 real

% give him the joints to computer analytical jacobian
joints = [q1, q2];


position = [
    l1*cos(q1) + l2*cos(q1 + q2);
    l1*sin(q1) + l2*sin(q1 + q2)
    q1 + q2
    ];

% Compute the analytical Jacobian
J = jacobian(position, joints);
disp('Analytical Jacobian matrix:')
disp(J)
