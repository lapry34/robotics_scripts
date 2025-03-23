function [Y_pi, pi] = linear_parametrization(M, c, g, q, dq, ddq, verbose)
    % LINEAR_PARAMETRIZATION_IMPROVED Computes the regressor matrix Y_pi and parameter vector pi
    % for the linear parametrization of robot dynamics, with improved handling of inertia terms
    %
    % Inputs:
    %   M: Symbolic inertia matrix M(q)
    %   c: Symbolic Coriolis and centrifugal vector c(q,dq)
    %   g: Symbolic gravity vector g(q)
    %   q: Joint position vector [q1; q2; ...]
    %   dq: Joint velocity vector [dq1; dq2; ...]
    %   ddq: Joint acceleration vector [ddq1; ddq2; ...]
    %   verbose: (optional) If true, prints detailed information
    %
    % Outputs:
    %   Y_pi: Regressor matrix such that M*ddq + c + g = Y_pi*pi
    %   pi: Vector of dynamic parameters
    
    % Check for optional verbose input
    if nargin < 7
        verbose = false;
    end
    
    % Get the number of joints
    n = length(q);
    
    if verbose
        disp(['Number of joints: ', num2str(n)]);
        disp('Computing linear parametrization...');
    end
    
    % Ensure q, dq, ddq are column vectors
    q = reshape(q, [], 1);
    dq = reshape(dq, [], 1);
    ddq = reshape(ddq, [], 1);
    
    % Compute the dynamic equation: M*ddq + c + g
    % Ensure all components have correct dimensions
    if ~isequal(size(c), [n, 1])
        c = reshape(c, n, 1);
    end
    
    if ~isequal(size(g), [n, 1])
        g = reshape(g, n, 1);
    end
    
    % Check if M has the correct dimensions
    if ~isequal(size(M), [n, n])
        error('Inertia matrix M must be an n×n matrix where n is the number of joints');
    end
    
    % Compute dynamic equation
    dynamic_eq = M*ddq + c + g;
    
    % Step 1: Collect all symbolic parameters from the equations
    all_vars = symvar([M(:); c(:); g(:)]);
    
    % Filter out kinematic variables (q, dq, ddq)
    kinematic_vars = [q; dq; ddq];
    sym_params = setdiff(all_vars, symvar(kinematic_vars));
    
    % Step 2: Create parameter vector
    pi = sym_params(:);
    
    % Step 3: Initialize the regressor matrix
    Y_pi = sym(zeros(n, length(pi)));
    
    % Step 4: Extract coefficients directly from dynamic equations
    for i = 1:n  % For each equation
        for j = 1:length(pi)  % For each parameter
            param = pi(j);
            
            % Create a function of the parameter
            eq_i = dynamic_eq(i);
            
            % Try to extract the coefficient using differentiation
            % This works when parameters appear linearly
            Y_pi(i, j) = diff(eq_i, param);
        end
    end
    
    % Step 5: Simplify the regressor matrix
    Y_pi = simplify(Y_pi);
    
    % Step 6: Check for zero columns (parameters that don't appear in the model)
    zero_cols = [];
    for j = 1:size(Y_pi, 2)
        if all(Y_pi(:, j) == 0)
            zero_cols = [zero_cols, j];
        end
    end
    
    if ~isempty(zero_cols) && verbose
        disp(['Found ', num2str(length(zero_cols)), ' zero columns (unused parameters)']);
    end
    
    % Step 7: Verify the parametrization
    verification = simplify(Y_pi * pi - dynamic_eq);
    
    if all(verification == 0)
        if verbose
            disp('VERIFICATION PASSED: Y_pi * pi equals the original dynamic equation.');
        end
    else
        if verbose
            warning('VERIFICATION FAILED: Y_pi * pi does not equal the original dynamic equation.');
            disp('Difference:');
            disp(verification);
        end
        
        % Additional verification step for troubleshooting
        if verbose
            disp('Attempting to identify missing terms...');
            
            % Examine each equation
            for i = 1:n
                eq_diff = verification(i);
                if eq_diff ~= 0
                    disp(['Missing terms in equation ', num2str(i), ':']);
                    disp(eq_diff);
                    
                    % Try to identify what parameters might be involved
                    vars_in_diff = symvar(eq_diff);
                    disp('Variables in missing terms:');
                    disp(vars_in_diff);
                end
            end
        end
    end
    
    % For compatibility with standard form
    pi_names = cell(length(pi), 1);
    for i = 1:length(pi)
        pi_names{i} = char(pi(i));
    end
    
    if verbose
        disp('Dynamic parameters:');
        for i = 1:length(pi)
            disp([num2str(i), ': ', pi_names{i}]);
        end
    end
end