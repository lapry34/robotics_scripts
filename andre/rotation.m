syms alpha beta gamma
%alpha = pi/4;
%beta=-pi/3;
%Rz = RotZ(alpha);
%Ry = RotY(beta);

Rz = RotZ(pi/2);
Rx = RotX(pi/4);
Ry = RotY(-pi/4);

Ri = Rz * Rx * Ry;

Rf = [
    0.8660,   -0.3536,   -0.3536;
    0.3536,    0.9330,   -0.0670;
    0.3536,   -0.0670,    0.9330;
];

Rif = transpose(Ri) * Rf;

[sol1, sol2] = rotm2eul(Rif, "YXY");
sol1
sol2

function R = RotX(angle)
    % Rotazione attorno all'asse X
    % Input: angle (può essere numerico o simbolico)
    % Output: matrice di rotazione 3x3 attorno a X

    R = [1,      0,           0;
         0, cos(angle), -sin(angle);
         0, sin(angle),  cos(angle)];
end

function R = RotY(angle)
    % Rotazione attorno all'asse Y
    % Input: angle (può essere numerico o simbolico)
    % Output: matrice di rotazione 3x3 attorno a Y

    R = [ cos(angle), 0, sin(angle);
                0, 1,      0;
         -sin(angle), 0, cos(angle)];
end

function R = RotZ(angle)
    % Rotazione attorno all'asse Z
    % Input: angle (può essere numerico o simbolico)
    % Output: matrice di rotazione 3x3 attorno a Z

    R = [cos(angle), -sin(angle), 0;
         sin(angle),  cos(angle), 0;
               0,           0,    1];
end