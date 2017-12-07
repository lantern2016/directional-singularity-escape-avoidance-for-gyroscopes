function [T_e, T_g, T_h] = get_T_ehg(y, config)
% Get auxiliary terms T_e, T_h, T_g
    
    % State variables    
    [sigma, sigma_dot, omega, gamma, gamma_dot, OMEGA] =  get_states(y, config);
    
    % Gimbal frame axes
    [A, G, H] = get_AGH(y, config);
    
    % Total inertia
    J_s = get_J_s(y, config);
    
    % Some inertia constants for readability
    J_T_f = config.model.J_T_f;
    J_T_g = config.model.J_T_g;
    J_T_h = config.model.J_T_h;
    J_W_h = config.model.J_W_h; 
    
    
    % The T_e term       
    T_e =   (J_T_h-J_T_f+J_T_g)*H*diag(A'*omega)*gamma_dot ...
           +(J_T_h-J_T_f-J_T_g)*A*diag(H'*omega)*gamma_dot ...
           +J_W_h*A*diag(OMEGA)*gamma_dot ...
           +J_W_h*A*diag(OMEGA)*G'*omega - J_W_h*G*diag(OMEGA)*A'*omega;
  
      
    % The T_h term 
    T_h = -config.model.J_W_h*diag(A'*omega)*gamma_dot;    
    
    % The T_g term 
    T_g = diag(A'*omega)*((J_T_h-J_T_f)*H'*omega+J_W_h*OMEGA);      
end
