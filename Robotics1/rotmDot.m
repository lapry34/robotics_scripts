function R_dot = rotmDot(R, omega)

    R_dot = R * skew(omega(1), omega(2), omega(3));
end