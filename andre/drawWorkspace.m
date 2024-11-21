clc; clear; close all;

% Lunghezza dei link
L = 1;

% Campionamento degli angoli
q1 = linspace(0, pi/2); % Angolo del primo giunto
q2 = linspace(-pi/2, pi/2);    % Angolo del secondo giunto

% Precalcolo dello spazio di lavoro
x_WS = [];
y_WS = [];
for i = 1:length(q1)
    for j = 1:length(q2)
        % Calcolo cinematica diretta
        x = L * cos(q1(i)) + L * cos(q1(i) + q2(j));
        y = L * sin(q1(i)) + L * sin(q1(i) + q2(j));
        x_WS = [x_WS, x];
        y_WS = [y_WS, y];
    end
end

% Genera figura
figure;
hold on;
axis equal;
xlabel('x');
ylabel('y');
title('Spazio di lavoro primario (WS1) con visualizzazione dei link');
grid on;

% Disegna lo spazio di lavoro
plot(x_WS, y_WS, 'b.', 'MarkerSize', 5); % Spazio di lavoro

% Limiti dello spazio
xlim([-2*L, 2*L]);
ylim([-2*L, 2*L]);

% Disegna i limiti dei link
theta = linspace(0, 2*pi, 100);
plot(2*L*cos(theta), 2*L*sin(theta), 'r--', 'LineWidth', 1); % Cerchio massimo
plot(L*cos(theta), L*sin(theta), 'b--', 'LineWidth', 1); % Cerchio del secondo link

% Inizializza grafici per i link
link1 = plot([0, 0], [0, 0], 'k-', 'LineWidth', 2); % Primo link
link2 = plot([0, 0], [0, 0], 'm-', 'LineWidth', 2); % Secondo link

% Inizializza punto finale
end_effector = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Genera animazione
for i = 1:length(q1)
    for j = 1:length(q2)
        % Calcolo cinematica diretta
        x1 = L * cos(q1(i)); % Fine del primo link
        y1 = L * sin(q1(i));
        
        x2 = x1 + L * cos(q1(i) + q2(j)); % Fine del secondo link
        y2 = y1 + L * sin(q1(i) + q2(j));
        
        % Aggiorna i link
        set(link1, 'XData', [0, x1], 'YData', [0, y1]); % Primo link
        set(link2, 'XData', [x1, x2], 'YData', [y1, y2]); % Secondo link
        
        % Aggiorna il punto finale
        set(end_effector, 'XData', x2, 'YData', y2);
        
        % Pause per animazione
        pause(0.01);
    end
end
