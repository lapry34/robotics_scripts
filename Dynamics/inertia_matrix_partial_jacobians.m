function D = inertia_matrix_partial_jacobians(DHTABLE, joints, masses, inertia_tensors)
    % ComputeInertiaMatrix Computes the robot's inertia matrix using recursive method
    % 
    % Inputs:
    %   DHTABLE: Denavit-Hartenberg table [alpha a d theta]
    %   joints: String of joint types ('R' for rotational, 'P' for prismatic)
    %   masses: Vector of link masses
    %   inertia_tensors: Cell array of 3x3 inertia tensors for each link in {body} frame
    %
    % Output:
    %   D: Inertia matrix (nxn, where n is number of joints)
    
    % Validate inputs
    N = size(DHTABLE, 1);
    assert(length(joints) == N, 'Number of joints must match DH table rows');
    assert(length(masses) == N, 'Number of masses must match DH table rows');
    assert(length(inertia_tensors) == N, 'Number of inertia tensors must match DH table rows');
    
    
    % Compute full geometric Jacobian for each joint
    [J_full, J_partial] = PartialGeometricJacobian(DHTABLE, joints);
    
    % Initialize inertia matrix
    D = zeros(N, N);
    
    % Compute inertia matrix components using Jacobian method
    for i = 1:N
        % Get partial Jacobian for current joint
        J_i = J_partial{i};
        
        % Extract translational and rotational Jacobian parts
        J_pi = J_i(1:3, :);  % Translational part
        J_oi = J_i(4:6, :);  % Rotational part
        
        % Current link's mass and inertia tensor
        m_i = masses(i);
        I_i = inertia_tensors{i};
        
        % Compute mass contribution
        D_mass = m_i * (J_pi' * J_pi);
        
        % Compute inertia contribution (rotational)
        % Transform inertia tensor to world frame using Jacobian
        D_inertia = J_oi' * I_i * J_oi;
        
        % Combine contributions
        D = D + D_mass + D_inertia;
    end

    D = simplify(D);
    
    % Optional: Symmetrize the matrix to ensure numerical stability
    D = (D + D') / 2;

    % Optional: Simplify the matrix
    D = simplify(D);
end
