function q_dot = projected_gradient(J, r_dot, q0_dot)
   
    I = eye(length(J));
    q_dot = simplify(pinv(J) * r_dot + (I - pinv(J) * J) * q0_dot);

end