function R_dot = RotationMatrixDerivative(R, angles, angles_dot)

    R_dot = zeros(3,3);

    for i = 1:3
        R_dot = R_dot + diff(R, angles(i)) * angles_dot(i);
    end

    R_dot = simplify(R_dot);

end