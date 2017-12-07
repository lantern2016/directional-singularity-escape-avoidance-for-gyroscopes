function tauref = get_tauref_from_Mref(Mref, y, config)
% Get gyroscopic reference moment tauref, based on the internal reference
% moment Mref, compensated for gyriscopic moments due to rotation of the
% whole body.

    % State variables    
    [sigma, sigma_dot, omega, gamma, gamma_dot, OMEGA] =  get_states(y, config);
    
    % Gimbal frame axes
    [A, G, H] = get_AGH(y, config);    
    
    % Gyroscopic moment due to rotation of spacecraft   
    d = config.model.Mu*(A*G'-G*A')*omega;
          
    % Reference moment to be produced by gimbal rotation
    tauref = Mref - d;
   
end