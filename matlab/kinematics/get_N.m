function N = get_N(sigma)
% Get the N matrix for the given MRP
   
    % Return N
    N = (1-sigma'*sigma)*eye(3)+2*tilde(sigma)+2*(sigma*sigma');
    
end

