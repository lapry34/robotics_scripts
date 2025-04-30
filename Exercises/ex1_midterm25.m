clear all;
clc;

digits(4);

addpath("../Dynamics/");
addpath("../Redundancy/");
addpath("../Robotics1/");


syms q1 q2 q3 q4 q5 real;

q = [q1; q2; q3; q4; q5];
% 5R planar robot, absolute angles unitary length links
p_ee = [
    cos(q1) + cos(q2) + cos(q3) + cos(q4) + cos(q5);
    sin(q1) + sin(q2) + sin(q3) + sin(q4) + sin(q5);
];

%tip of link 3;
p_3 = [
    cos(q1) + cos(q2) + cos(q3);
    sin(q1) + sin(q2) + sin(q3);
];

% tip of link 2;
p_2 = [
    cos(q1) + cos(q2);
    sin(q1) + sin(q2);
];

J_ee = jacobian(p_ee, q);
J_3 = jacobian(p_3, q);
J_2 = jacobian(p_2, q);


q_curr = [0; pi/3; -pi/4; pi/2; -pi/3];
J_ee_curr = subs(J_ee, q, q_curr);
J_3_curr = subs(J_3, q, q_curr);
J_2_curr = subs(J_2, q, q_curr);

r_dot_1 = [2; 3];
r_dot_2 = [2; -0.5];
r_dot_3 = [-1; 0];
r_dot_cell = {r_dot_1, r_dot_2, r_dot_3};

J_cell = {J_ee_curr, J_3_curr, J_2_curr};
% Task priority recursive algorithm

P = eye(4);
q_dot = zeros(4, 1);

q_dot_TP = task_priority_recursive(J_cell, r_dot_cell);
disp("Task Priority Recursive");
disp("q_dot_TP : ");
disp(vpa(q_dot_TP));

%compute the error for each task
r_dot_2_realized = J_2_curr*q_dot_TP;
r_dot_3_realized = J_3_curr*q_dot_TP;
r_dot_ee_realized = J_ee_curr*q_dot_TP;

disp("error first task (ee): ");
disp(norm((vpa(r_dot_1 - r_dot_ee_realized))));
disp("error second task (link 3): ");
disp(norm(vpa(r_dot_2 - r_dot_3_realized)));
disp("error third task (link 2): ");
disp(norm(vpa(r_dot_3 - r_dot_2_realized)));
%plot the error for each task

%task augmenting
disp("Task augmenting with first two tasks");

J_aug_2 = [J_ee_curr; J_3_curr];
J_aug_tot = [J_ee_curr; J_3_curr; J_2_curr];
r_dot_aug_2 = [r_dot_1; r_dot_3];
r_dot_aug_tot = [r_dot_1; r_dot_3; r_dot_2];
q_dot_aug_2 = pinv(J_aug_2)*r_dot_aug_2;
q_dot_aug_tot = pinv(J_aug_tot)*r_dot_aug_tot;
disp("q_dot_aug_2 : ");
disp(vpa(q_dot_aug_2));
disp("rank of J_aug_2 : ");
disp(rank(J_aug_2));
%compute error for each task
r_dot_2_realized = J_2_curr*q_dot_aug_2;
r_dot_3_realized = J_3_curr*q_dot_aug_2;
r_dot_ee_realized = J_ee_curr*q_dot_aug_2;
disp("error first task (ee): ");
disp(norm((vpa(r_dot_1 - r_dot_ee_realized))));
disp("error second task (link 3): ");
disp(norm(vpa(r_dot_2 - r_dot_3_realized)));
disp("error third task (link 2): ");
disp(norm(vpa(r_dot_3 - r_dot_2_realized)));
%task augmenting with all tasks
disp("Task augmenting with all tasks");
disp("rank of J_aug_tot : ");
disp(rank(J_aug_tot));
disp("q_dot_aug_tot : ");
disp(vpa(q_dot_aug_tot));
%compute the error for each task
r_dot_2_realized = J_2_curr*q_dot_aug_tot;
r_dot_3_realized = J_3_curr*q_dot_aug_tot;
r_dot_ee_realized = J_ee_curr*q_dot_aug_tot;
disp("error first task (ee): ");
disp(norm((vpa(r_dot_1 - r_dot_ee_realized))));
disp("error second task (link 3): ");
disp(norm(vpa(r_dot_2 - r_dot_3_realized)));
disp("error third task (link 2): ");
disp(norm(vpa(r_dot_3 - r_dot_2_realized)));

T = [1 0 0 0 0;
     1 1 0 0 0;
     1 1 1 0 0;
     1 1 1 1 0;
     1 1 1 1 1;
     ];

theta_dot = inv(T)*q_dot_TP;
disp("theta_dot : ");
disp(vpa(theta_dot));