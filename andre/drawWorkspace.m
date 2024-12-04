clc; clear; close all;

% Lunghezza fissa tra il joint revolute e l'end-effector
L = 1;

% Limiti dei giunti prismatici
q1_min = -2.5; % Limite minimo del primo giunto prismatico
q1_max = 2.5;  % Limite massimo del primo giunto prismatico
q2_min = -2.5; % Limite minimo del secondo giunto prismatico
q2_max = 2.5;  % Limite massimo del secondo giunto prismatico

% Campionamento dei giunti
q1 = linspace(q1_min, q1_max); % Spostamento del primo giunto prismatico
q2 = linspace(q2_min, q2_max); % Spostamento del secondo giunto prismatico
q3 = linspace(0, 2*pi);        % Angolo del giunto rotazionale

% Precalcolo dello spazio di lavoro
x_WS = [];
y_WS = [];
for i = 1:length(q1)
    for j = 1:length(q2)
        for k = 1:length(q3)
            % Calcolo cinematica diretta
            x = q1(i) + q2(j) * cos(q3(k)); % Coordinata x
            y = q2(j) * sin(q3(k));         % Coordinata y
            
            % Aggiungi il punto nello spazio di lavoro
            x_WS = [x_WS, x];
            y_WS = [y_WS, y];
        end
    end
end

% Genera figura
figure;
hold on;
axis equal;
xlabel('x');
ylabel('y');
title('Spazio di lavoro del manipolatore RPP con limiti ai giunti');
grid on;

% Disegna lo spazio di lavoro
plot(x_WS, y_WS, 'b.', 'MarkerSize', 5);

% Limiti dello spazio
xlim([q1_min + q2_min, q1_max + q2_max]);
ylim([q2_min, q2_max]);

% Inizializza grafici per i link
link1 = plot([0, 0], [0, 0], 'k-', 'LineWidth', 2); % Primo link
link2 = plot([0, 0], [0, 0], 'm-', 'LineWidth', 2); % Secondo link

% Inizializza punto finale
end_effector = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Genera animazione
for i = 1:length(q1)
    for j = 1:length(q2)
        for k = 1:length(q3)
            % Calcolo cinematica diretta
            x1 = q1(i); % Fine del primo link
            y1 = 0;
            
            x2 = x1 + q2(j) * cos(q3(k)); % Fine del secondo link
            y2 = q2(j) * sin(q3(k));
            
            % Aggiorna i link
            set(link1, 'XData', [0, x1], 'YData', [0, y1]); % Primo link
            set(link2, 'XData', [x1, x2], 'YData', [y1, y2]); % Secondo link
            
            % Aggiorna il punto finale
            set(end_effector, 'XData', x2, 'YData', y2);
            
            % Pause per animazione
            pause(0.01);
        end
    end
end
