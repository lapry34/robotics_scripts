function res = proj_NullSpace(J)
    I = eye(length(J));
    res = (I-pinv(J)*J);
    res = simplify(res);
end

