function w_q = function_w(q) %obstacle
    p_O = [-0.55 0 0.2]';
    T = kuka_directkinematics(q);
    x = T(1:3,4);
    w_q = norm(min(x - p_O));
end