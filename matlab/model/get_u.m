function u = get_u(t, y, gamma_dot_c, config)
% Get the gimbal and spin torque for the current reference gamma_dot
    
    % State variables    
    [sigma, sigma_dot, omega, gamma, gamma_dot, OMEGA] =  get_states(y, config);

    % Desired accelerations
    gamma_dotdot_d = config.controller.k_gamma * (gamma_dot_c                 - gamma_dot);
    OMEGA_dot_d    = config.controller.k_OMEGA * (config.controller.OMEGA_c - OMEGA    );

    % System terms
    [T_e, T_g, T_h] = get_T_ehg(y, config);
    
    % Spacecraft inertia
    J_s = get_J_s(y, config);
    J_T_g = config.model.J_T_g;
    J_W_h = config.model.J_W_h;
    
    % External moment
    M_e = get_M_e(y, config);

    % Gimbal frame vectors
    [A, G, H] =get_AGH(y, config);

    % Body acceleration corresponding to desired gimbal and flywheel
    % accelerations
    omega_dot = J_s\(M_e - T_e - cross(omega,J_s*omega) - J_T_g*G*gamma_dotdot_d - J_W_h*H*OMEGA_dot_d);

    % Gimbal torque
    u_g = J_T_g*gamma_dotdot_d + J_T_g*G'*omega_dot - T_g;

    % Flywheel torque
    u_h = J_W_h*OMEGA_dot_d    + J_W_h*H'*omega_dot - T_h;
    
    % Stacked control input
    u = [u_g; u_h];
end