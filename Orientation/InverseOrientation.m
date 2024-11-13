% Clear workspace
clear variables;
clc;

% Define the rotation matrix R
R = [
-2, 2, -1;
2, 1, -2;
-1, -2, -2;
];
R = R / 3;

% Call the function to get the axis and angles
[r_1, r_2, theta_1, theta_2] = inverseRotationMatrix(R);

% Display the results
fprintf("First solution:\n");
fprintf("r_1: \n");
disp(r_1);
fprintf("theta_1: %.2f radians\n", theta_1);
fprintf("theta_1: %.2f*PI \n", theta_1/pi);
fprintf("theta_1: %.2f degrees\n", theta_1*180/pi)

fprintf("\nSecond solution (opposite):\n");
fprintf("r_2: \n");
disp(r_2);
fprintf("theta_2: %.2f radians\n", theta_2);
fprintf("theta_2: %.2f*PI \n", theta_2/pi);
fprintf("theta_2: %.2f degrees\n", theta_2*180/pi)


function [r_1, r_2, theta_1, theta_2] = inverseRotationMatrix(R)
    % This function computes the axis r and angles theta from a given 
    % rotation matrix R using the inverse of Rodrigues' rotation formula.
    %
    % Inputs:
    % R - a 3x3 rotation matrix
    %
    % Outputs:
    % r_1, theta_1 - the first solution for the axis and angle
    % r_2, theta_2 - the second solution (opposite rotation)

    tolerance_s = 1e-6;  % Set a small tolerance for sine values
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

    % Compute sine values
    s_1 = sin(theta_1);

    % Check condition theta_1 -> 0 (undefined)
    if abs(theta_1) < tolerance_theta
        error("Undefined! Theta approaches 0.");

    % Check conditions if s_1 -> 0 and theta_1 -> pi
    elseif (abs(s_1) < tolerance_s && abs(abs(theta_1) - pi) < tolerance_theta)
        r_1 = [sqrt(R_11 + 1); sqrt(R_22 + 1); sqrt(R_33 + 1)] / sqrt(2);
    else
        % Create vector r from cross product components
        r_1 = [R_32 - R_23; R_13 - R_31; R_21 - R_12] / (2 * sin(theta_1));
    end
    
    % Check if rxry = R12/2, rxrz = R13/2, ryrz = R23/2 holds, to choose
    % the sign of each component of r_1
    if sign(r_1(1) * r_1(2)) ~= sign(R_12 / 2)
        r_1(1) = -r_1(1);
    end
    if sign(r_1(1) * r_1(3)) ~= sign(R_13 / 2)
        r_1(3) = -r_1(3);
    end
    if sign(r_1(2) * r_1(3)) ~= sign(R_23 / 2)
        r_1(2) = -r_1(2);
    end
    
    % Second solution (opposite axis)
    r_2 = -r_1;
    theta_2 = -theta_1;

end
