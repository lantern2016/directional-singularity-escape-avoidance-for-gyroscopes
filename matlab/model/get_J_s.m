function J_s = get_J_s(y, config)
% Get total spacecraft inertia
    
    % Gimbal frame axes
    [A, G, H] = get_AGH(y, config);
    
    % Spacecraft body
    J_B = diag([config.model.J_B_x, config.model.J_B_y, config.model.J_B_z]);
    
    % Total inertia
    J_s = J_B + config.model.J_T_f*(A*A')+config.model.J_T_g*(G*G')+config.model.J_T_h*(H*H');  
end
