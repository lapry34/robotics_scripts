function Jw_pinv = weighted_pinv(J, W)
    
    W_inv = inv(W);
    
    Jw_pinv = simplify(W_inv * J' * inv(J * W_inv * J'));
    
end