%% DH transformation matrices and direct kinematics of a serial robot (SCARA example)
%% 5 Nov 2012 (DH frames assigned as in lecture slides 09_DirectKinematics, A. De Luca)

clear all
clc

%% Define symbolic variables

syms alpha d a theta L L1 L2 L3 L4 a1 a2 a3 a4 d1 d2 d3 d4 H q0 q1 q2 q3 q4 gamma real

%% number and type of joints of SCARA

N=4;
joints_str = 'RRPR';

assert(N == length(joints_str), "Mismatch between N and length of joints_str");

%% Insert DH table of parameters of SCARA
% DONOT USE N AS SYM
% alpha a d theta
DHTABLE = [
    -pi/2 0 d1 q1;
    0 a2 0 q2;
    -pi/2 0 0 q3;
    0 0 d4 q4;
];


         
%% Build the general Denavit-Hartenberg trasformation matrix

TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0             sin(alpha)             cos(alpha)            d;
          0               0                      0                   1];

%% Build transformation matrices for each link

A = cell(1,N);
for i = 1:N
    alpha = DHTABLE(i,1);
    a = DHTABLE(i,2);
    d = DHTABLE(i,3);
    theta = DHTABLE(i,4);
    A{i} = subs(TDH);
end

% % Display the rotation matrix R_i
% i = 1;
% R_i = A{i}(1:3, 1:3);
% disp("R_" + i + " = [" + join(string(R_i(1,:)'), " ") + "; " ...
%                    + join(string(R_i(2,:)'), " ") + "; " ...
%                    + join(string(R_i(3,:)'), " ") + "];");

%% Direct kinematics

disp(['N=',num2str(N)])


T = eye(4);

p_vec = [];
z_vec = [];

p_i = T(1:3, 4);
z_i = T(1:3, 3);
disp("p_0 = [" + join(string(p_i), "; ") + "];");
disp("z_0 = [" + join(string(z_i), "; ") + "];");
p_vec = [p_vec, p_i];
z_vec = [z_vec, z_i];

for i=1:N 
    T = T*A{i};
    T = simplify(T);
    
    % disp p_i and z_i
    p_i = T(1:3, 4);
    z_i = T(1:3, 3);
    disp("p_" + i + " = [" + join(string(p_i), "; ") + "];");
    disp("z_" + i + " = [" + join(string(z_i), "; ") + "];");
    p_vec = [p_vec, p_i];
    z_vec = [z_vec, z_i];
end

disp("__________________________________")


% output TN matrix
% output ON position
% output xN axis
% output yN axis
% output zN axis
T0N = T
p = T(1:3,4)
n=T(1:3,1)
s=T(1:3,2)
a=T(1:3,3)

%% Geometric Jacobian

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
disp("Jacobian matrix:");
disp(J);

disp("Determinant:");
if N == 6
    det_jo = det(JO);
else
    det_jo = det(JO * JO.');
end
disp(simplify(det_jo));

%% change in order to the question
% 1. Find the singularities from det
% 2. Study the singularities changing the code
% ex. 
% det = sin q2 -> q2 = 0, pi in the code
% then compute the null space in order to study the singularities

% null(JP) gives the base B s.t. for each q_dot=alpha*B -> v = JP*q_dot = 0
% null(JO) gives the base B s.t. for each q_dot=alpha*B -> w = JO*q_dot = 0
% null(J.') gives the base B s.t. for each F=alpha*B -> tau = J.' * F = 0

% disp(simplify(null(JO)));