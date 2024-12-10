clear variables;
clc

syms l1 l2 d2 d3 q1 q2 real

% Joints and DH parameters
joints = [q1, q2, d3];
paramOrder = {'alpha', 'a', 'd', 'theta'};
DH = [
    0, -pi/2, 0, q1;
    0, pi/2, d2, q2;
    0, 0, d3, 0;
];


% Compute transformations
[T_end, T_all] = directKinematics(DH, paramOrder);

% Display the final transformation
disp('Final transformation matrix T (base to end-effector):')
disp(T_end)

% Position of the end-effector
position = T_end(1:3, 4);
disp('Position of the end-effector:')
disp(position)

% Compute the geometric Jacobian
J_geom = geometricJacobian(T_all, joints, 'RRP');
disp('Geometric Jacobian matrix:')
disp(J_geom)

% Compute the analytical Jacobian
J_analytical = jacobian(position, joints);
disp('Analytical Jacobian matrix:')
disp(J_analytical)

% Functions
function [T, T_matrices] = directKinematics(DH, paramOrder)
    % DH: nx4 matrix where each row is [p1, p2, p3, p4] in the specified order
    % paramOrder: 1x4 cell array specifying the order of DH parameters e.g {'alpha', 'a', 'd', 'theta'}
    
    % Number of joints
    n = size(DH, 1);
   
    % Map parameter names to indices
    paramMap = containers.Map({'alpha', 'a', 'd', 'theta'}, 1:4);
    paramIdx = cellfun(@(x) paramMap(x), paramOrder);
    
    % Initialize transformation matrix as identity
    T = eye(4);
    T_matrices = sym(zeros(4, 4, n));  % 3D array to store all intermediate transformations
    
    for i = 1:n
        % Extract DH parameters in the specified order
        alpha = DH(i, paramIdx(1));
        a = DH(i, paramIdx(2));
        d = DH(i, paramIdx(3));
        theta = DH(i, paramIdx(4));

        % Compute the transformation matrix for the current joint
        Ti = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
              sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0, sin(alpha), cos(alpha), d;
              0, 0, 0, 1];
        
        % Update the total transformation matrix
        T = T * Ti;
        T = simplify(T);

        % Store the intermediate transformation matrix
        T_matrices(:, :, i) = T;
    end
end

function J = geometricJacobian(T_all, joints, type_joints)
    % Computes the geometric Jacobian matrix using intermediate transformations
    % T_all: 3D array of intermediate transformation matrices
    % joints: symbolic vector of joint variables
    
    % Number of transformations
    n = size(T_all, 3);
    % Initialize Jacobian components
    Jv = sym(zeros(3, n));  % Linear velocity Jacobian
    Jw = sym(zeros(3, n));  % Angular velocity Jacobian
    
    % Extract base origin and z-axis
    z_prev = [0; 0; 1];
    
    for i = 1:n
        % Extract the origin and z-axis of the current transformation
        j = n-i+1;
        T = T_all(:, :, j);
        z = T(1:3, 3);

        T_end = T_all(:, :, n);
        end_effector = T_end(1:3, 4);

        % Check if the current joint is prismatic or revolute
        if type_joints(j) == 'R'
            % Revolute joint
            Jv(:, j) = diff(end_effector, joints(j)); %ci sta quella non differenziale ma non riesco a farla funzionare
            Jw(:, j) = z_prev;
        else
            % Prismatic joint
            Jv(:, j) = z_prev;
            Jw(:, j) = [0; 0; 0];
        end
        
        % Update the previous origin and z-axis
        z_prev = z;
    end
    
    % Combine linear and angular parts into the full Jacobian
    J = [Jv; Jw];
    J = simplify(J);
end