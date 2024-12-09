% Import symbolic library and define variables
clear variables;
clc

syms l1 l2 d2 d3 q1 q2 real

% give him the joints to computer analytical jacobian
joints = [q1, q2, d3];

% alpha and a always constant!!!!
% Define the custom order of parameters
paramOrder = {'alpha', 'a', 'd', 'theta'};

% Define DH parameters for two joints
% Note: Each row is [alpha, a, d, theta] in the specified order
% parameters are used for transformations in inverse order as in the book (theta, d, a, alpha)
DH = [
    0, l1, 0, q1;   % First joint (prismatic)
    0, l2, 0, q2;  % Second joint (revolute)
];

DH = [
    0, -pi/2, 0, q1;
    0, pi/2, d2, q2;
    0, 0, d3, 0;
];

% Compute the direct kinematics
T = directKinematics(DH, paramOrder);

% Display the transformation matrix
disp('Transformation matrix T from base to end-effector:')
disp(T)


% Extract the end-effector position from T
position = T(1:3, 4);
disp('End-effector position:')
disp(position)

% Compute the analytical Jacobian
J = jacobian(position, joints);
disp('Analytical Jacobian matrix:')
disp(J)

% Compute the geometric Jacobian
J = geometricJacobian(DH, paramOrder, joints);
disp('Geometric Jacobian matrix:')
disp(J)

% Functions
function T = directKinematics(DH, paramOrder)
    % DH: nx4 matrix where each row is [p1, p2, p3, p4] in the specified order
    % paramOrder: 1x4 cell array specifying the order of DH parameters e.g {'alpha', 'a', 'd', 'theta'}
    
    % Number of joints
    n = size(DH, 1);
   
    % Map parameter names to indices
    paramMap = containers.Map({'alpha', 'a', 'd', 'theta'}, 1:4);
    paramIdx = cellfun(@(x) paramMap(x), paramOrder);
    
    % Initialize transformation matrix as identity
    T = eye(4);
    
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
    end
end

function J = geometricJacobian(DH, paramOrder, joints)
    % Computes the geometric Jacobian matrix from DH parameters
    % DH: nx4 DH parameter table
    % paramOrder: Order of parameters in the DH table
    
    % Number of joints
    n = size(DH, 1);
    
    % Initialize transformation matrix and Jacobian matrix
    T = eye(4);
    Jv = sym(zeros(3, n));  % Linear velocity Jacobian
    Jw = sym(zeros(3, n));  % Angular velocity Jacobian
    
    % Map parameter names to indices
    paramMap = containers.Map({'alpha', 'a', 'd', 'theta'}, 1:4);
    paramIdx = cellfun(@(x) paramMap(x), paramOrder);
    
    % Base origin and orientation
    origin = [0; 0; 0];
    z_prev = [0; 0; 1];
    
    for i = 1:n
        % Extract DH parameters in the specified order
        alpha = DH(i, paramIdx(1));
        a = DH(i, paramIdx(2));
        d = DH(i, paramIdx(3));
        theta = DH(i, paramIdx(4));
        
        % Compute transformation matrix for current joint
        Ti = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
              sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0, sin(alpha), cos(alpha), d;
              0, 0, 0, 1];
        
        % Update total transformation matrix
        T = T * Ti;
        
        % Extract current origin and z-axis
        o = T(1:3, 4);
        z = T(1:3, 3);

        % Compute Jacobian columns
        if ismember(d, joints)  % Prismatic joint
            Jv(:, i) = z_prev;
            Jw(:, i) = [0; 0; 0];
        else  % Revolute joint
            Jv(:, i) = cross(z_prev, o - origin);
            Jw(:, i) = z_prev;
        end
        
        % Update origin and z-axis
        origin = o;
        z_prev = z;
    end
    
    % Combine linear and angular parts
    J = [Jv; Jw];
    J = simplify(J);

    % we have to add the last column to the precedent one and so on iteratively
    % TODO: only when the joints are revolute?????????? update
    for i = n-1:-1:1
        J(1:3, i) = J(1:3, i) + J(1:3, i+1);
    end

    J = simplify(J);

end