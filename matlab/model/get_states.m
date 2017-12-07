function [sigma, sigma_dot, omega, gamma, gamma_dot, OMEGA] =  get_states(y, config)
% Decompose the state y into useful variables

    % Read all states
    sigma     = y(config.model.id.sigma    , :);
    sigma_dot = y(config.model.id.sigma_dot, :);
    gamma     = y(config.model.id.gamma    , :);
    gamma_dot = y(config.model.id.gamma_dot, :);
    OMEGA     = y(config.model.id.OMEGA    , :);  
    
    % Convert angular velocity
    omega     = sigmadot2omega(sigma_dot, sigma);
end
