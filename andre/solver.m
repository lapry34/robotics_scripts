% Definizione delle variabili simboliche
syms q1 q2 q3 Px Py phi a1 a3 real

% Sistema di equazioni
eq1 = Px == a1 * cos(q1) - q2 * sin(q1) + a3 * cos(q1 + q3);
eq2 = Py == a1 * sin(q1) + q2 * cos(q1) + a3 * sin(q1 + q3);
eq3 = phi == q1 + q3;

% Uso di solve per trovare le soluzioni
solutions = solve([eq1, eq2, eq3], [q1, q2, q3]);
q1_solution = solutions.q1;
q2_solution = solutions.q2;
q3_solution = solutions.q3;
% Estrazione delle soluzioni
%q1_solution = simplify(solutions.q1); % Soluzione per q1
%q2_solution = simplify(solutions.q2); % Soluzione per q2
%q3_solution = simplify(solutions.q3); % Soluzione per q3

% Mostrare i risultati
disp('Soluzione per q1:');
disp(q1_solution);

disp('Soluzione per q2:');
disp(q2_solution);

disp('Soluzione per q3:');
disp(q3_solution);
