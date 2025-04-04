clc;

clear all;
digits(4);

addpath("./Redundancy/")

syms q1 q2 q3 q4 real;
syms q1_dot q2_dot q3_dot q4_dot real;
syms q1_ddot q2_ddot q3_ddot q4_ddot real;


p_e = [cos(q1) + cos(q2) + cos(q3) + cos(q4);
        sin(q1) + sin(q2) + sin(q3) + sin(q4)];


q_0 = [0; pi/6; -pi/3; -pi/3];

J_e = jacobian(p_e, [q1; q2; q3; q4]);
J_e = simplify(J_e);

p_t = [cos(q1) + cos(q2);
        sin(q1) + sin(q2)];

J_t = jacobian(p_t, [q1; q2]);
J_t = simplify(J_t);

v_e_d = [0.4330; -0.75];
v_t_d = [-0.5; 0.8660];

q_dot_e = pinv( subs(J_e, [q1; q2; q3; q4], q_0) ) * v_e_d;
q_dot_t = pinv( subs(J_t, [q1; q2], q_0(1:2)) ) * v_t_d;

v_e_obtained = subs(J_e, [q1; q2; q3; q4], q_0) * q_dot_e;
v_t_obtained = subs(J_t, [q1; q2], q_0(1:2)) * q_dot_t;

% PART 1: AT BEST MINIMIZING THE NORM.
disp("Part 1: AT BEST MINIMIZING THE NORM")

disp("q_dot_e: ")
disp(vpa(q_dot_e))

disp("q_dot_t: ")
disp(vpa(q_dot_t))

disp("error on e: ")
disp(vpa(v_e_d - v_e_obtained))
disp("error on e norm: ")
disp(vpa(norm(v_e_d - v_e_obtained)))
disp("error on t: ")
disp(vpa(v_t_d - v_t_obtained))
disp("error on t norm: ")
disp(vpa(norm(v_t_d - v_t_obtained)))

% PART 2: TASK AUGMENTATION, BOTH TOGETHER AT BEST
disp("Part 2: TASK AUGMENTATION, BOTH TOGETHER AT BEST")

J_aug = [J_e; [J_t, zeros(2)]];

J_aug = simplify(J_aug);

disp("J_aug: ")
disp(vpa(J_aug))

disp("determinant of augmented jacobian in q_0: ")
disp(vpa(det(subs(J_aug, [q1; q2; q3; q4], q_0))))
disp("rank of augmented jacobian in q_0: ")
disp(vpa(rank(subs(J_aug, [q1; q2; q3; q4], q_0))))

J_aug_inv = pinv( subs(J_aug, [q1; q2; q3; q4], q_0) );
q_dot_aug = pinv( subs(J_aug, [q1; q2; q3; q4], q_0) ) * [v_e_d; v_t_d];
v_e_obtained_aug = subs(J_e, [q1; q2; q3; q4], q_0) * q_dot_aug;
v_t_obtained_aug = subs(J_t, [q1; q2], q_0(1:2)) * q_dot_aug(1:2);
disp("q_dot_aug: ")
disp(vpa(q_dot_aug))

disp("error on e: ")
disp(vpa(v_e_d - v_e_obtained_aug))
disp("error on e norm: ")
disp(vpa(norm(v_e_d - v_e_obtained_aug)))

disp("error on t: ")
disp(vpa(v_t_d - v_t_obtained_aug))
disp("error on t norm: ")
disp(vpa(norm(v_t_d - v_t_obtained_aug)))

%PART 3 TASK PRIORITY, v_e FIRST
disp("Part 3: TASK PRIORITY, v_e FIRST")

J_t = jacobian(p_t, [q1; q2; q3; q4]);
J_t = simplify(J_t);

J_e_subs = subs(J_e, [q1; q2; q3; q4], q_0);
J_t_subs = subs(J_t, [q1; q2], q_0(1:2));

q_dot_p1 = task_priority(J_e_subs, v_e_d, J_t_subs, v_t_d);
v_e_obtained_p1 = subs(J_e, [q1; q2; q3; q4], q_0) * q_dot_p1;
v_t_obtained_p1 = subs(J_t, [q1; q2], q_0(1:2)) * q_dot_p1;

disp("q_dot_p1: ")
disp(vpa(q_dot_p1))

disp("error on e: ")
disp(vpa(v_e_d - v_e_obtained_p1))
disp("error on e norm: ")
disp(vpa(norm(v_e_d - v_e_obtained_p1)))
disp("error on t: ")
disp(vpa(v_t_d - v_t_obtained_p1))
disp("error on t norm: ")
disp(vpa(norm(v_t_d - v_t_obtained_p1)))

%PART 4 TASK PRIORITY, v_t FIRST
disp("Part 4: TASK PRIORITY, v_t FIRST")

q_dot_p2 = task_priority(J_t_subs, v_t_d, J_e_subs, v_e_d);
v_e_obtained_p2 = subs(J_e, [q1; q2; q3; q4], q_0) * q_dot_p2;
v_t_obtained_p2 = subs(J_t, [q1; q2], q_0(1:2)) * q_dot_p2;
disp("q_dot_p2: ")
disp(vpa(q_dot_p2))

disp("error on e: ")
disp(vpa(v_e_d - v_e_obtained_p2))
disp("error on e norm: ")
disp(vpa(norm(v_e_d - v_e_obtained_p2)))
disp("error on t: ")
disp(vpa(v_t_d - v_t_obtained_p2))
disp("error on t norm: ")
disp(vpa(norm(v_t_d - v_t_obtained_p2)))