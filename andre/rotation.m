%syms alpha beta
%alpha = pi/4;
%beta=-pi/3;
%Rz = RotZ(alpha);
%Ry = RotY(beta);



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