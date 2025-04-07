% Define symbolic variables
syms q1 q2 dq1 dq2 ddq1 ddq2 real
syms m1 m2 l1 l2 g0 Izz1 Izz2 real

% Define joint variables
q = [q1; q2];
dq = [dq1; dq2];
ddq = [ddq1; ddq2];

% Create your dynamic model (example for a 2R planar robot)
% ... code to create M, c, and g ...

% Optional: Create a friction model
friction.Fv = [0.1; 0.1]; % Viscous friction coefficients
friction.Fc = [0.2; 0.2]; % Coulomb friction coefficients

% Generate the linear parametrization


[Y, a, Y_symbolic] = linear_parametrization(M, c, g, q, dq, ddq, friction);