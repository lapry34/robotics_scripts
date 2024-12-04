clear variables;
clc 


syms delta
% Define axis r and angle theta
r = [0; -sqrt(2)/2; sqrt(2)/2];  % Rotation about the Z-axis
theta = pi/6;   % 90 degrees rotation

% Call the function to compute the rotation matrix R
R = axisAngleToRotationMatrix(r, theta);

% Display the result
fprintf("R: \n")
disp(R)




function R = axisAngleToRotationMatrix(r, theta)
    % This function computes the rotation matrix R given an axis r and an
    % angle theta (in radians) using the Rodrigues' rotation formula.
    %
    % Inputs:
    % r     - a 3x1 unit vector representing the axis of rotation
    % theta - the angle of rotation (in radians)
    %
    % Output:
    % R     - the resulting 3x3 rotation matrix

    % Ensure r is a column vector
    r = r(:);

    % Cross-product matrix S of vector r
    S = [0, -r(3), r(2);
         r(3), 0, -r(1);
         -r(2), r(1), 0];

    % Precompute cos(theta) and sin(theta)
    c_t = cos(theta);
    s_t = sin(theta);

    % Compute r * r^T (outer product)
    rrT = r * r.';

    % Compute the rotation matrix using Rodrigues' rotation formula
    R = rrT + (eye(3) - rrT) * c_t + S * s_t;
end
