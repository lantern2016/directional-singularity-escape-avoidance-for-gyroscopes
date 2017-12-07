function Mref = get_Mref_for_constant_tau(t, y, config)
% Get internal moment such, that after compensation for d, we actually get
% a constant tau. This is a rather artifical situation, but useful for testing

    % State variables    
    [sigma, sigma_dot, omega, gamma, gamma_dot, OMEGA] =  get_states(y, config);
    
    % Gimbal frame axes
    [A, G, H] = get_AGH(y, config);    
    
    % Gyroscopic moment due to rotation of spacecraft   
    d = config.model.Mu*(A*G'-G*A')*omega;
          
    % Reference moment to be produced by gimbal rotation
    Mref = config.controller.constant_output + d;
   
end
