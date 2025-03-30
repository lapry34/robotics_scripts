function [w_x, w_y, w_z] = rotmToOmega(R, q, dq)

    R_dot = diff(R, q) * dq;
    R_dot = simplify(R_dot);
    S_w = R_dot * R';
    w_x = S_w(3, 2);
    w_y = S_w(1, 3);
    w_z = S_w(2, 1);
    w_x = simplify(w_x);
    w_y = simplify(w_y);
    w_z = simplify(w_z);
