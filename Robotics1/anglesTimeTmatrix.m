
function T = anglesTimeTmatrix(angles, sequence, fixed)
    % angles: symbolic or numeric angles [alpha, beta, gamma]
    % sequence: rotation sequence (e.g., "ZYX")
    % fixed: boolean indicating fixed or rotating frame

    sequence = char(sequence);
    
    R1 = eye(3);
    R2 = Rot(angles(1), sequence(1));
    R3 = R2 * Rot(angles(2), sequence(2));

    n1 = numberAxis(sequence(1));
    n2 = numberAxis(sequence(2));
    n3 = numberAxis(sequence(3));

    if fixed 
        T = [R3(:,n3), R2(:,n2), R1(:,n1)];
    else 
        T = [R1(:,n1), R2(:,n2), R3(:,n3)];
    end

end

function n = numberAxis(axis)
    switch axis
        case 'X'
            n = 1;
        case 'Y'
            n = 2;
        case 'Z'
            n = 3;
        otherwise
            n = 0;
    end
end



function R = RotX(angle)
    R = [1,      0,           0;
         0, cos(angle), -sin(angle);
         0, sin(angle),  cos(angle)];
end

function R = RotY(angle)
    R = [ cos(angle), 0, sin(angle);
                0, 1,      0;
         -sin(angle), 0, cos(angle)];
end

function R = RotZ(angle)
    R = [cos(angle), -sin(angle), 0;
         sin(angle),  cos(angle), 0;
               0,           0,    1];
end

function R = Rot(angle, axis)
    switch axis
        case 'X'
            R = RotX(angle);
        case 'Y'
            R = RotY(angle);
        case 'Z'
            R = RotZ(angle);
        otherwise
            R = eye(3);
    end
end