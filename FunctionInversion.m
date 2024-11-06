clear variables;
clc;


% Define the vector-valued function and its Jacobian
f = @(x) [x(1)^2 + x(2); x(1) + x(2)^2 - 1];  % Example vector-valued function


delta = 1e-5;  % Small step size

% Target value for which we want to find the inverse
y = [3; 1];  % Example target vector

% Initial guess for x
x0 = [1; 1];
x = x0;

% Gradient Descent parameters
alpha = 0.01;  % Learning rate for gradient descent
tol = 1e-6;    % Tolerance for convergence
max_iter = 10000;

% Gradient Descent
for i = 1:max_iter
    error = f(x) - y;
    if norm(error) < tol
        fprintf('Gradient Descent converged in %d iterations\n', i);
        break;
    end
    x = x - alpha * numJ(f,x,delta)' * error;  % Gradient Descent update
end

% If Gradient Descent did not converge, display a warning
if i == max_iter
    warning('Gradient Descent did not converge within the maximum iterations');
end


% Display the final solution
fprintf('The inverse solution of f(x) = [%f; %f] is approximately x = [%f; %f]\n', y(1), y(2), x(1), x(2));

y = f(x);
fprintf('f(x_sol) = [%f; %f]\n', y(1), y(2));

%reset initial guess
x = x0;

% Newton's Method
for j = 1:max_iter
    error = f(x) - y;
    if norm(error) < tol
        fprintf('Newton Method converged in %d iterations\n', j);
        break;
    end
    x = x - numJ(f,x,delta) \ error;  % Newton's Method update using the Jacobian inverse
end

% If Newton's Method did not converge, display a warning
if j == max_iter
    warning('Newton Method did not converge within the maximum iterations');
end

% Display the final solution
fprintf('The inverse solution of f(x) = [%f; %f] is approximately x = [%f; %f]\n', y(1), y(2), x(1), x(2));

y = f(x);
fprintf('f(x_sol) = [%f; %f]\n', y(1), y(2));

function J = numJ(f, x, delta)
    n = numel(x);
    fx = f(x);
    m = numel(fx);
    J = zeros(m, n);
    
    for i = 1:n
        x_step = x;
        x_step(i) = x_step(i) + delta;
        J(:, i) = (f(x_step) - fx) / delta;  % Forward difference
    end
end

