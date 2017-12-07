function output = get_gamma_dot_c_Wie2005(t, y, tau_r, config)
% Get the control input for the current state and desired reference moment,
% using the o-DSR method presented in:
% Wie, B., "Singularity Escape/Avoidance Steering Logic for Control Moment Gyro Systems," Journal of Guidance, Control, and Dynamics, Vol. 28, No. 5, Sept-Oct. 2005, pp. 948-956. doi: 10.2514/1.10136

    % Initialize output structure
    output.gamma_dot_c         = zeros(config.model.nCMGs,1);
    output.gamma_dot_c_bounded = zeros(config.model.nCMGs,1);
    output.gamma_dot_p         = zeros(config.model.nCMGs,1);
    output.gamma_dot_d         = zeros(config.model.nCMGs,1);
    output.gamma_dot_e         = zeros(config.model.nCMGs,1);
    output.gamma_dot_d_Vspace  = zeros(config.model.nCMGs,1);
    output.gamma_dot_e_Vspace  = zeros(config.model.nCMGs,1);
    output.tau_c_Uspace        = zeros(config.model.outputdimension,1);    
    output.tau_d_Uspace        = zeros(config.model.outputdimension,1);
    output.tau_e_Uspace        = zeros(config.model.outputdimension,1);    
    output.tau_error_Uspace    = zeros(config.model.outputdimension,1);    
    output.tau_c               = zeros(config.model.outputdimension,1);
    output.tau_p               = zeros(config.model.outputdimension,1);
    output.tau_d               = zeros(config.model.outputdimension,1);
    output.tau_e               = zeros(config.model.outputdimension,1);   
    output.tau_error           = zeros(config.model.outputdimension,1);   
    
    % For initialization, only return this 'empty' output structure
    if(isempty(t)) 
        return;
    end  
    
    % Number of CMGS
    n = config.model.nCMGs;        
      
    % Singularity measures and system matrices. These have already been
    % adjusted for the 2D and 3D case, depending on the selected
    % configuration.
    [delta, A, G, H, U, S, V] = get_singularity_measures(t, y, tau_r, config);
  
    % Also reduce the reference moment to the appropriate dimension
    tau_r = tau_r(1:config.model.outputdimension);

    epsilon_Wie = config.controller.k_epsilon0*sin(config.controller.omega_epsilon*t+config.controller.phi_i);
    
    % Create the perturbation matrices either for the planar or 3D application
    if(config.model.planar_torques_only)
        V_wie = delta.lambda*[1 epsilon_Wie; epsilon_Wie 1];
    else
        V_wie = delta.lambda*[1 epsilon_Wie(3) epsilon_Wie(2); epsilon_Wie(3) 1 epsilon_Wie(1); epsilon_Wie(2) epsilon_Wie(1) 1];
    end
    
    W_wie = diag(config.controller.W_wie)+triu(ones(n),1)*delta.lambda+tril(ones(n),-1)*delta.lambda;    
    
       
    % No explicit perturbation rates, projected or otherwise:
    output.gamma_dot_d = zeros(n,1);

    %%%% Now that the perturbation/weighing matrices are known
    %%%% we can compute the control law     
    
    % Pseudoinverse with dithering for singularity escape and avoidance (Wie, 2005)
    A_hash = W_wie*A'/(A*W_wie*A'+V_wie);   
    A_sr   = diag(config.controller.W_wie)*A'/(A*diag(config.controller.W_wie)*A'+delta.lambda*eye(config.model.outputdimension));   

    % Controller output
    output.gamma_dot_c = 1/config.model.Mu*A_hash*tau_r;
    
    % It is also useful to study the separate terms in the above sum:
    output.gamma_dot_p        = 1/config.model.Mu*A_sr*tau_r;
    output.gamma_dot_e        = output.gamma_dot_c-output.gamma_dot_p;  
    
    % Speed/moment reference debug info
    output.tau_c = config.model.Mu*A*output.gamma_dot_c ;
    output.tau_p = config.model.Mu*A*output.gamma_dot_p ;
    output.tau_d = config.model.Mu*A*output.gamma_dot_d ; 
    output.tau_e = config.model.Mu*A*output.gamma_dot_e ;
    
    output.gamma_dot_d_Vspace = V'*output.gamma_dot_d;
    output.gamma_dot_e_Vspace = V'*output.gamma_dot_e;
    

    % Output error (neglecting speed tracking error)
    output.tau_error = tau_r-output.tau_c;
    
    % Output components in U Space
    output.tau_d_Uspace = U'*output.tau_d;
    output.tau_e_Uspace = U'*output.tau_e;      
    output.tau_c_Uspace = U'*output.tau_c;    
    output.tau_error_Uspace = U'*output.tau_error;      
    
    % Bound the actual gimbal rates (if we do reach these limits, the error
    % expressions in SVD form given above no longer hold)    
    output.gamma_dot_c_bounded = max(min(config.controller.maxGimbalRate,output.gamma_dot_c),-config.controller.maxGimbalRate);
    
%     if(t == 0.38 || t == 0.40)
%         t
% %         A
% %         y(config.model.id.gamma)*180/pi
% %         output.gamma_dot_c_bounded
% %        A_hash = W_wie*A'/(A*W_wie*A'+V_wie)
% %         inv((A*W_wie*A'+V_wie))
%         keyboard
%     end
    
end

