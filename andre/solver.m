% Definizione delle variabili simboliche
syms q1 q2 q3 q4 
% ESERCIZIO 5, SOSTITUIRE I VALORI CON QUELLI DEL VETTORE
L = 1.5;
Px = 0;
Py = 1.5;
Pz = 4;
alpha = 0; 

% Sistema di equazioni
eq1 = Px == sin(q1+q3) + L * cos(q1)*cos(q4);
eq2 = Py == -cos(q1+q3)+ L * sin(q1)*cos(q4);
eq3 = Pz == q2 + L * sin(q4);
eq4 = alpha == q4;

% Uso di solve per trovare le soluzioni
solutions = solve([eq1, eq2, eq3, eq4], [q1, q2, q3, q4]);
q1_solution = solutions.q1;
q2_solution = solutions.q2;
q3_solution = solutions.q3;
q4_solution = solutions.q4;

% Mostrare i risultati
disp('Soluzione per q1:');
disp(q1_solution);

disp('Soluzione per q2:');
disp(q2_solution);

disp('Soluzione per q3:');
disp(q3_solution);

disp('Soluzione per q4');
disp(q4_solution);
