clear all;
clc;

digits(4);

addpath("../Dynamics/");
addpath("../Redundancy/");

syms g0 d m Ic positive;
syms q q_dot q_ddot real;

p_c = [
    d * cos(q);
    d * sin(q);
];

v_c = diff(p_c,q)*q_dot;

T = 0.5 * m * (v_c' * v_c) + 0.5 * q_dot * Ic * q_dot;

U = - m *g0 * d*cos(q);
L = T - U;
model = diff(diff(L, q_dot), q_dot)*q_ddot - diff(L, q);
disp("model: ");
simplify(model)

[M, c, S, G] = compute_dynamic_model(q, q_dot, T, U, false);
disp("M + n");
disp(simplify(M *q_ddot + c + G));

%rest to rest trajectory 
q_in = 0;
q_fin = pi;

syms t T k real;
k = 1;
tau = k*t/T;


delta_q = q_fin - q_in;
q_tau = q_in + delta_q * (-2 * tau^3 + 3 * tau^2);
q_dot_tau = diff(q_tau,t)  * k/T;
q_ddot_tau = diff(q_dot_tau,t) * k/T;

q_tau = simplify(q_tau);
q_dot_tau = simplify(q_dot_tau);
q_ddot_tau = simplify(q_ddot_tau);

disp("q_ddot_tau: ");
disp(q_ddot_tau);

% needed torque equation
torq = M*q_ddot_tau + c + G;
torq = subs(torq, q, q_tau)
torq = simplify(torq);

%find maximum torque with derivative
torq_diff = diff(torq, t) * k/T

% Add numerical values
m_val = 1;    % kg
d_val = 0.66;  % m
Ic_val = 1.07; % kg*m^2
g0_val = 9.81; % m/s^2
umax = 18;     % Nm


disp("Ic + m*d^2: ");
disp(Ic_val + m_val*d_val^2)


% Substitute symbolic expressions with numerical values
M_num = subs(M, [m, d, Ic, g0], [m_val, d_val, Ic_val, g0_val]);
G_num = subs(G, [m, d, g0], [m_val, d_val, g0_val]);
c_num = subs(c, [m, d, Ic], [m_val, d_val, Ic_val]);

% Create time vector
t_vec = linspace(0, 1, 100);
T_val = 1;
k_val = 1;

% Initialize arrays
u = zeros(size(t_vec));
q_traj = zeros(size(t_vec));
qd_traj = zeros(size(t_vec));
qdd_traj = zeros(size(t_vec));

% Compute trajectory and required torque
for i = 1:length(t_vec)
    % Substitute values directly into symbolic expressions
    q_traj(i) = double(subs(q_tau, [t, T, k], [t_vec(i), T_val, k_val]));
    qd_traj(i) = double(subs(q_dot_tau, [t, T, k], [t_vec(i), T_val, k_val]));
    qdd_traj(i) = double(subs(q_ddot_tau, [t, T, k], [t_vec(i), T_val, k_val]));
    
    % Compute required torque with substituted q and q_dot
    G_num_i = double(subs(G_num, q, q_traj(i)));
    c_num_i = double(subs(c_num, [q, q_dot], [q_traj(i), qd_traj(i)]));
    u(i) = double(M_num*qdd_traj(i) + c_num_i + G_num_i);
end

% Plot results for original trajectory (k=1, T=1)
figure(1);
subplot(2,1,1)
plot(t_vec, q_traj, 'b', 'LineWidth', 2)
hold on
plot(t_vec, qd_traj, 'r', 'LineWidth', 2)
plot(t_vec, qdd_traj, 'g', 'LineWidth', 2)
grid on
legend({'Position', 'Velocity', 'Acceleration'})
title('Trajectory (T=1s, k=1)')

subplot(2,1,2)
plot(t_vec, u, 'b', 'LineWidth', 2)
hold on
plot(t_vec, umax*ones(size(t_vec)), 'r--')
plot(t_vec, -umax*ones(size(t_vec)), 'r--')
grid on
legend({'Required Torque', 'Torque Limits'})
title('Required Torque (T=1s, k=1)')

% Find maximum torque
max_abs_torque = max(abs(u));
fprintf('Maximum required torque at T=1s: %.4f Nm\n', max_abs_torque);

% Define a function to compute max torque for a given time
calculate_max_torque = @(T_test) compute_max_torque_for_time(T_test, q_tau, q_dot_tau, q_ddot_tau, ...
                                                  M_num, c_num, G_num, t_vec, k_val, q, q_dot, t, T, k);

% Binary search to find minimum time
T_min_search = 0.5;  % Initial lower bound
T_max_search = 2.0;  % Initial upper bound
tolerance = 1e-4;    % Tolerance for convergence

% Binary search loop
fprintf('Searching for minimum feasible time...\n');
iterations = 0;
max_iterations = 20; % Prevent infinite loops

while (T_max_search - T_min_search) > tolerance && iterations < max_iterations
    iterations = iterations + 1;
    T_mid = (T_min_search + T_max_search) / 2;
    max_torque_mid = calculate_max_torque(T_mid);
    
    if max_torque_mid > umax
        % Too fast, need to increase time
        T_min_search = T_mid;
    else
        % Can go faster, reduce time
        T_max_search = T_mid;
    end
    
    fprintf('Iteration %d: T = %.4f s, max torque = %.4f Nm\n', iterations, T_mid, max_torque_mid);
end

T_min = T_max_search;  % Conservative choice
fprintf('Correct minimum time T_min = %.4f s\n', T_min);
k_min = 1; % No need to scale the trajectory shape

% Parameters for minimum time plot
T_custom = T_min;  % Use the correct minimum time
k_custom = 1;      % No scaling needed

% Create a new time vector for custom parameters
t_vec_custom = linspace(0, T_custom, 100);
u_custom = zeros(size(t_vec_custom));
q_traj_custom = zeros(size(t_vec_custom));
qd_traj_custom = zeros(size(t_vec_custom));
qdd_traj_custom = zeros(size(t_vec_custom));

% Compute trajectory and torque with custom parameters
for i = 1:length(t_vec_custom)
    % Substitute custom values into symbolic expressions
    q_traj_custom(i) = double(subs(q_tau, [t, T, k], [t_vec_custom(i), T_custom, k_custom]));
    qd_traj_custom(i) = double(subs(q_dot_tau, [t, T, k], [t_vec_custom(i), T_custom, k_custom]));
    qdd_traj_custom(i) = double(subs(q_ddot_tau, [t, T, k], [t_vec_custom(i), T_custom, k_custom]));
    
    G_num_i = double(subs(G_num, q, q_traj_custom(i)));
    c_num_i = double(subs(c_num, [q, q_dot], [q_traj_custom(i), qd_traj_custom(i)]));
    u_custom(i) = double(M_num*qdd_traj_custom(i) + c_num_i + G_num_i);
end

% Plot results with custom parameters
figure(2);
subplot(2,1,1)
plot(t_vec_custom, q_traj_custom, 'b', 'LineWidth', 2)
hold on
plot(t_vec_custom, qd_traj_custom, 'r', 'LineWidth', 2)
plot(t_vec_custom, qdd_traj_custom, 'g', 'LineWidth', 2)
grid on
legend({'Position', 'Velocity', 'Acceleration'})
title(['Trajectory with T = ' num2str(T_custom) 's, k = ' num2str(k_custom)])

subplot(2,1,2)
plot(t_vec_custom, u_custom, 'b', 'LineWidth', 2)
hold on
plot(t_vec_custom, umax*ones(size(t_vec_custom)), 'r--')
plot(t_vec_custom, -umax*ones(size(t_vec_custom)), 'r--')
grid on
legend({'Required Torque', 'Torque Limits'})
title(['Required Torque with T = ' num2str(T_custom) 's, k = ' num2str(k_custom)])

% Check maximum torque with custom parameters
max_abs_torque_custom = max(abs(u_custom));
fprintf('Maximum torque with minimum time (T=%.4f, k=%.2f): %.4f Nm\n', T_custom, k_custom, max_abs_torque_custom);

% Helper function to calculate maximum torque for a given time
function max_torque = compute_max_torque_for_time(T_test, q_tau, q_dot_tau, q_ddot_tau, ...
                                         M_num, c_num, G_num, t_vec, k_val, q, q_dot, t, T, k)
    u_test = zeros(size(t_vec));
    
    for i = 1:length(t_vec)
        q_val = double(subs(q_tau, [t, T, k], [t_vec(i), T_test, k_val]));
        qd_val = double(subs(q_dot_tau, [t, T, k], [t_vec(i), T_test, k_val]));
        qdd_val = double(subs(q_ddot_tau, [t, T, k], [t_vec(i), T_test, k_val]));
        
        G_num_i = double(subs(G_num, q, q_val));
        c_num_i = double(subs(c_num, [q, q_dot], [q_val, qd_val]));
        u_test(i) = double(M_num*qdd_val + c_num_i + G_num_i);
    end
    
    max_torque = max(abs(u_test));
end