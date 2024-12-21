
%% Main function for SCARA direct kinematics
clear all; clc;

syms L L1 L2 L3 L4 a1 a2 a3 a4 d1 d2 d3 d4 H q0 q1 q2 q3 q4 gamma real

% Define symbolic variables

% Define DH table and type of joints
N = 4; % Number of joints

joints_str = 'RRPR';
joints_var = [q1, q2, q3, q4];

DHTABLE = [
    -pi/2 0 d1 q1;
    0 a2 0 q2;
    -pi/2 0 0 q3;
    0 0 d4 q4;
];

assert(N == length(joints_str), "Mismatch between N and length of joints_str");
assert(N == size(DHTABLE, 1), "Mismatch between N and number of rows in DHTABLE");
assert(N == length(joints_var), "Mismatch between N and length of joints_var");

% Build transformation matrices
A = build_transformation_matrices(DHTABLE);

% Perform direct kinematics
[p_vec, z_vec, T0N] = direct_kinematics(A, N);

% Extract position and orientation of the end-effector
disp("Position of the end-effector:");
p_ee = T0N(1:3, 4);
disp(p_ee);

% Compute Jacobian
J_geom = compute_geometric_jacobian(p_vec, z_vec, joints_str, N);
J_analytical = simplify(jacobian(p_ee, joints_var));
disp("Analytical Jacobian matrix:");
disp(J_analytical);

% Analyze singularities
J_P = J_geom(1:3, :);
J_O = J_geom(4:6, :);
analyze_singularities(J_analytical, joints_var);


function A = build_transformation_matrices(DHTABLE)
    %% Build transformation matrices for each link
    N = size(DHTABLE, 1);
    A = cell(1, N);

    for i = 1:N
        alpha = DHTABLE(i, 1);
        a = DHTABLE(i, 2);
        d = DHTABLE(i, 3);
        theta = DHTABLE(i, 4);

        TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
                sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                  0             sin(alpha)             cos(alpha)            d;
                  0               0                      0                   1];

        A{i} = TDH;
    end
end

function [p_vec, z_vec, T0N] = direct_kinematics(A, N)
    %% Perform direct kinematics
    T = eye(4);
    N = length(A);


    p_i = T(1:3, 4);
    z_i = T(1:3, 3);
    disp("p_0 = [" + join(string(p_i), "; ") + "];");
    disp("z_0 = [" + join(string(z_i), "; ") + "];");
    p_vec = [p_i];
    z_vec = [z_i];

    for i = 1:N
        T = T * A{i};
        T = simplify(T);
        % disp p_i and z_i
        p_i = T(1:3, 4);
        z_i = T(1:3, 3);
        disp("p_" + i + " = [" + join(string(p_i), "; ") + "];");
        disp("z_" + i + " = [" + join(string(z_i), "; ") + "];");
        p_vec = [p_vec, p_i];
        z_vec = [z_vec, z_i];
    end

    T0N = T;
    fprintf("\n\n__________________________________\n\n");
    disp("Final Transformation Matrix T0N:");
    disp(T0N);
end

function J = compute_geometric_jacobian(p_vec, z_vec, joints_str, N)
    %% Compute the geometric Jacobian
    JP = [];
    JO = [];

    for i = 1:N
        p_i = p_vec(:, i);
        z_i = z_vec(:, i);
        if joints_str(i) == 'R'
            JP = [JP, cross(z_i, p_vec(:, end) - p_i)];
            JO = [JO, z_i];
        else
            JP = [JP, z_i];
            JO = [JO, [0; 0; 0]];
        end
    end

    J = [JP; JO];
    J = simplify(J);
    disp("Geometric Jacobian matrix:");
    disp(J);
end

function analyze_singularities(J, joints_var)
    %% Analyze singularities based on Jacobian determinant

    %check if the jacobian is square
    [rows, cols] = size(J);

    det_J = 0;

    if rows ~= cols
        det_J = simplify(det(J * J'));
    else 
        det_J = simplify(det(J));
    end
    disp("Determinant of the Jacobian:");
    disp(det_J);

    singular_points = solve(det_J == 0, joints_var);
    disp("Singular configurations:");
    disp(singular_points);
end
