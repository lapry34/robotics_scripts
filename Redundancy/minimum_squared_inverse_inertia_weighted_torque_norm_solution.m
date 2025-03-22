function tau = minimum_squared_inverse_inertia_weighted_torque_norm_solution(J,  J_dot, q_dot, M, r_ddot, c, g)
    %Inputs:
    %   J : (m x n) jacobian matrix of the task 
    %   J_dot : (m x n) time derivative of the jacobian matrix
    %   q_dot : (m x 1) velocity of the joints
    %   M : (n x n) inertia matrix
    %   c :  (n x 1) Coriolis and centrifugal forces
    %   g : (n x 1) gravity vector
    %Output:
    %   tau : (n x 1) minimum (squared inverse inertia weighted) torque norm solution for dynamic redundancy
    
    n = c + g;

    M_inv = inv(M);
    tau = simplify(M * pinv(J) * (r_ddot - J_dot * q_dot + J * M_inv *n));

end