%Inverse problem for Rotation
clear variables;
clc
% Define the rotation matrix R%
R = [-1, 0, 0; 0, -1/sqrt(2), -1/sqrt(2); 0, -1/sqrt(2), 1/sqrt(2)];

tolerance_s = 1e-6;  % Set a small tolerance for s
tolerance_theta = 1e-3;  % Set a small tolerance for theta

% Elements of the rotation matrix
R_12 = R(1,2);
R_21 = R(2,1);

R_13 = R(1,3);
R_31 = R(3,1);

R_23 = R(2,3);
R_32 = R(3,2);

R_11 = R(1,1);
R_22 = R(2,2);
R_33 = R(3,3);

% Compute intermediate variables y, x
y = sqrt((R_12 - R_21)^2 + (R_13 - R_31)^2 + (R_23 - R_32)^2);
x = R_11 + R_22 + R_33 - 1;

% Compute angles theta_1 and (opposite) theta_2
theta_1 = atan2(y, x);
theta_2 = atan2(-y, x);

% Compute sine values
s_1 = sin(theta_1);
s_2 = sin(theta_2);

%check condition theta -> 0
if abs(theta_1) < tolerance_theta
    fprintf("Undefined!!! theta -> 0")
% Check conditions if s_1/s_2 -> 0 and theta -> pi
elseif (abs(s_1) < tolerance_s && abs(abs(theta_1) - pi) < tolerance_theta) || (abs(s_2) < tolerance_s && abs(abs(theta_2) - pi) < tolerance_theta)
    r_1 = [sqrt(R_11 + 1); sqrt(R_22 + 1); sqrt(R_33 + 1)];
    r_1 = r_1 / 4;
    r_2 = -r_1;

    %print the result
    fprintf("r_1: \n")
    disp(r_1)
    
    fprintf("r_2: \n")
    disp(r_2)
else
    % Create vector r with consistent dimensions (3x1 vector)
    r = [R_32 - R_23; R_13 - R_31; R_21 - R_12];
    r = r / (2 * sin(theta_1));

    %print the result
    fprintf("r: \n")
    disp(r)
end
