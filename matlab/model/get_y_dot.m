function [y_dot, M_internal, tau] = get_y_dot(t, y, u, config)
% Calculate the state change
    
    % State variables    
    [sigma, sigma_dot, omega, gamma, gamma_dot, OMEGA] =  get_states(y, config);

    % Gimbal frame axes
    [A, G, H] = get_AGH(y, config);

    % Body inertia
    J_B = diag([config.model.J_B_x, config.model.J_B_y, config.model.J_B_z]);
    
    % Total spacecraft inertia
    J_s = get_J_s(y, config);
    
    % Extract inertia constants for readability
    J_T_g = config.model.J_T_g;
    J_W_h = config.model.J_W_h;

    % Generalized mass matrix
    n = config.model.nCMGs;
    Mass = [J_s,      J_T_g*G,        J_W_h*H       ;
            J_T_g*G', J_T_g*eye(n,n), zeros(n,n)    ;
            J_W_h*H', zeros(n,n),     J_W_h*eye(n,n)];

    % Auxiliary input terms
    [T_e, T_g, T_h] = get_T_ehg(y, config);
    M_e             = get_M_e(y, config);
    
    % Solve for the accelerations
    accelerations = Mass\([M_e;u]+[-T_e- cross(omega,J_s*omega);T_g;T_h]);
    omega_dot    = accelerations(config.model.id.omega);
    gamma_dotdot = accelerations(config.model.id.gamma); 
    OMEGA_dot    = accelerations(config.model.id.psi  ); 
    
    % Convert body acceleration to MRP acceleration
    sigma_dotdot = 1/4*get_N(sigma)*omega_dot + 1/4*get_N_dot(sigma,sigma_dot)*omega;
    
    % The state change vector
    y_dot = [sigma_dot; gamma_dot; OMEGA; sigma_dotdot; gamma_dotdot; OMEGA_dot];
    
    % With the known acceleration, the moment follows from euler's law
    % for the main body. We take the negative to find the moment acting
    % on the CMG, compensated for gravity which also acts
    M_acceleration = J_B*omega_dot + cross(omega,J_B*omega);        
    M_internal     = get_M_e(y, config) - M_acceleration;
    
    %The tau approximation 
    tau = config.model.Mu*A*gamma_dot;

    
end
