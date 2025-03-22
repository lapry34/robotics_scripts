

function [M_p, c_p, g_p, u_p] = coordinate_transformation(M_q, c_q, g_q, u_q, q_dot, J, J_dot)
    % M: mass matrix
    % c: coriolis matrix
    % g: gravity matrix
    % tau: torque vector
    % q: joint position vector

    %p = subs(f, q);
    J_inv = inv(J);

    p_dot = J*q_dot;

    M_p = J_inv' * M_q * J_inv;
    c_p = J_inv' * c_q - M_p*J_dot*J_inv*p_dot; % J_inv*p_dot = q_dot? simplify?
    g_p = J_inv'*g_q;

    u_p = J_inv'*u_q;

end

