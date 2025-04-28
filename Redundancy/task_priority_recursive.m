function q_dot = task_priority_recursive(J_cell, r_dot_cell)


    n_joints = size(J_cell{1}, 2);
    q_dot = zeros(n_joints, 1);
    P = eye(n_joints);

    for i = 1:length(J_cell)
        J = J_cell{i};
        r_dot = r_dot_cell{i};

        JP = J*P;
        
        % check if JP is close to singularity using singular values
        sv = svd(JP);
        tol = 1e-8;
        if any(sv < tol)
            fprintf('Warning: JP is close to singularity at iteration %d\n', i);
            JP_inv = J_damped_least_squares(JP, tol);
        else
            JP_inv = pinv(JP);
        end

        q_dot = q_dot + JP_inv*(r_dot - J*q_dot);
        P = P - JP_inv*JP;
    end 

    q_dot = simplify(q_dot);