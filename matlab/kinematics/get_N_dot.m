function N = get_N_dot(sigma,sigma_dot)
% Get the time derivative of N matrix for the given MRP
   
    % Return N_dot
    N = -2*sigma_dot'*sigma*eye(3)+2*tilde(sigma_dot)+4*(sigma_dot*sigma');
    
end

