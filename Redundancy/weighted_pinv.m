function Jw_pinv = weighted_pinv(J, W)
    
    Jw_pinv = simplify(inv(W) * J' * inv(J * inv(W) * J'));
    
end