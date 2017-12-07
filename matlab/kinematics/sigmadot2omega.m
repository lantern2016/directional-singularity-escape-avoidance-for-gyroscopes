function omega = sigmadot2omega(sigmadot, sigma)
% Convert MRP time derivative to angular velocity

    % Allow this function to accept omegas/sigmas for multiple time instances
    nSteps = size(sigmadot,2);
    omega = zeros(3,nSteps);
    
    % Do the conversion for each time step
    for i = 1:nSteps
        %Current sigma value
        sigma_now = sigma(:,i);
        
        % N matrix
        N = get_N(sigma_now);

        % Get the MRP time derivative
        omega(:,i) = 4/(1+sigma_now'*sigma_now)^2*N'*sigmadot(:,i);        
    end
end

