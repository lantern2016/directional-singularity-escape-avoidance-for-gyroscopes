function H = get_H(R, r)
% Create homeogeneous transformation matrix
    H = [R, r; 0, 0, 0, 1];    
end

