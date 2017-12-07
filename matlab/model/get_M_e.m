function M_e = get_M_e(y,config)
    % Get external moment such as gravity, if any.
    
    % State variables    
    [sigma, sigma_dot, omega, gamma, gamma_dot, OMEGA] =  get_states(y, config);    
    
    % Rotation matrix from inertial to body frame:
    R = get_R_MRP(sigma);
    
    % Gravity force in inertial frame and body frame:
    F_in = [0;0;-1]*config.model.m_B*config.model.gravity;
    F_B  = R*F_in;
    
    % Gravity moment in body frame
    M_e = cross(config.model.pendulumCOM,F_B) + config.model.M_e_disturbance;
end