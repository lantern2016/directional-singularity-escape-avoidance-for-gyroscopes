function sigmadot = omega2sigmadot(omega, sigma)
% Convert angular velocity to MRP time derivative

    % Allow this function to accept omegas/sigmas for multiple time instances
    nSteps = size(omega,2);
    sigmadot = zeros(3,nSteps);
    
    % Do the conversion for each time step
    for i = 1:nSteps
        % N matrix
        N = get_N(sigma(:,i));

        % Get the angular velocity
        sigmadot(:,i) = 1/4*N*omega(:,i);        
    end
end

