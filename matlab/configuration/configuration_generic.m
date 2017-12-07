function config = configuration_generic(config)
% Extend configuration structure with properties that hold
% regardless of the selected platform
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Generic Model Properties 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Output dimensionality (task size)
    
    % In case of planar torques, it is assumed without loss of generality,
    % that the gimbal axes are all g_i = [0;0;1], which implies selecting
    % the body frame in such a way that this is the case.
    
    if(config.model.planar_torques_only)
        config.model.outputdimension = 2;
    else
        config.model.outputdimension = 3;
    end  

    % Gimbal assembly properties
    config.model.J_T_f = config.model.J_G_f + config.model.J_W_f;
    config.model.J_T_g = config.model.J_G_g + config.model.J_W_g;
    config.model.J_T_h = config.model.J_G_h + config.model.J_W_h;
    
    % Spin angular momentum per wheel
    config.model.Mu = config.model.J_W_h * config.controller.OMEGA_c;
    
    % Variable indexes for easy reference
    config.model.id.sigma = (1:3)'; % In actual simulation state
    config.model.id.omega = (1:3)'; % In temporary euler-newton state
    
    config.model.id.gamma = 3+(1:config.model.nCMGs)';                 % Gimbal angles
    config.model.id.psi   = config.model.id.gamma+config.model.nCMGs; % Wheel angles
    
    % Total number of configuration coordinates
    config.model.nCoordinates = 3 + config.model.nCMGs*2;
    config.model.nStates = config.model.nCoordinates*2;
    
    % Indexes of their time derivatives
    config.model.id.sigma_dot = config.model.id.sigma + config.model.nCoordinates;
    config.model.id.omega_dot = config.model.id.omega + config.model.nCoordinates; 
    config.model.id.gamma_dot = config.model.id.gamma + config.model.nCoordinates; % Gimbal rates
    config.model.id.OMEGA     = config.model.id.psi   + config.model.nCoordinates; % Wheel rates
    
    % Index of MRPs
    config.model.id.MRPs = [config.model.id.sigma; config.model.id.sigma_dot];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Generic Simulation Properties 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Initial orientation and angular velocity
    sigma0   = config.simulation.InitialMRP;
    sigma_dot0 = 1/4*get_N(sigma0)*config.simulation.InitialAngularVelocity;

    % Initial gimbal rates, angles
    config.simulation.initialGimbalAngles = config.simulation.initialGimbalAnglesDegrees/180*pi;    
    gamma0     = config.simulation.initialGimbalAngles;
    psi0       = zeros(config.model.nCMGs,1);
    gamma_dot0 = config.simulation.initialGimbalRates;
    OMEGA0     = ones(config.model.nCMGs,1)*config.controller.OMEGA_c;

    % Complete Initial state
    q0     = [sigma0    ; gamma0    ; psi0  ];
    q_dot0 = [sigma_dot0; gamma_dot0; OMEGA0];
    y0     = [q0; q_dot0];
    
    % If the angular velocity was not specified (NaN), choose it such that the
    % system's total angular momentum is zero.
    if(any(isnan(sigma_dot0)))  
        [~, ~, H0] = get_AGH(y0, config);
        config.simulation.InitialAngularVelocity = -inv(get_J_s(y0, config))*H0*config.model.J_W_h*OMEGA0;
        sigma_dot0 = 1/4*get_N(sigma0)*config.simulation.InitialAngularVelocity;
        y0(config.model.id.sigma_dot) = sigma_dot0;
    end

    % Initial state, mapped to shadow parameters
    [y0(config.model.id.MRPs),~] = shadowMRP(y0(config.model.id.MRPs ));
    config.simulation.y0 = y0;   
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Maneuver Properties (if specified)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Reference moment (in case of constant tau simulation)
    config.controller.constant_output = get_unit_vector(config.controller.target_direction)*config.controller.tauref_max;  
    
    %Target MRP (in case of maneuver simulation)
    config.controller.targetMRP        = axis2MRP(config.controller.target_direction, config.controller.target_degrees*pi/180);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Generic Timing Properties 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

    if(config.simulation.sampling_time > config.controller.sampling_time)
        error('Simulation sampling time must not be greater than controller sampling time.');
    end

    config.simulation.nSimStepsPerControlSample = round(config.controller.sampling_time/config.simulation.sampling_time);

    if(config.controller.sampling_time/config.simulation.sampling_time ~= config.simulation.nSimStepsPerControlSample)
        error('Controller sampling time must be integer multiple of simulation sampling time.');
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Generic Animation Properties 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Transformation matrix from body frame to inertial frame
    H_inertial_body = @(y) get_Homogenous(get_R_MRP(y(config.model.id.sigma))',[0;0;0])*get_Homogenous(eye(3),config.model.pendulumCOM);
    
    % Plot object properties for each body and angular momentum vector 
    config.animation.bodies{1}.dimensions = config.animationsetup.body_dimensions;
    config.animation.bodies{1}.color      = config.animationsetup.body_color;
    config.animation.bodies{1}.sides      = config.animationsetup.body_sides;
    config.animation.bodies{1}.H          = H_inertial_body;
    config.animation.bodies{1}.up         = 'z';

     for i = 1:config.model.nCMGs
         
        % Index of this gimbal angle in the state vector
        gamma_id = 3 + i;
        
        % Transformation from gimbal frame to body frame
        R_body_gimbal = @(y) [-config.model.H0(:,i)*sin(y(gamma_id))+config.model.F0(:,i)*cos(y(gamma_id)), config.model.G0(:,i), config.model.H0(:,i)*cos(y(gamma_id))+config.model.F0(:,i)*sin(y(gamma_id))];
        H_gimbal_inertial = @(y) H_inertial_body(y) *get_Homogenous( R_body_gimbal(y),config.animationsetup.gimbal_com(1:3,i));

        % Wheel object
        bodynr = 3*i-1;
        config.animation.bodies{bodynr}.dimensions = config.animationsetup.wheel_dimensions;
        config.animation.bodies{bodynr}.color      = config.animationsetup.wheel_color;
        config.animation.bodies{bodynr}.sides      = config.animationsetup.wheel_sides;
        config.animation.bodies{bodynr}.H          = H_gimbal_inertial;
        config.animation.bodies{bodynr}.up         = 'z';
        
        % Momentum vector object
        vector_length = config.animationsetup.wheel_dimensions(1);
        vector_width  = vector_length/20;
        config.animation.bodies{bodynr+1}.dimensions = [vector_width;vector_width;vector_length];
        config.animation.bodies{bodynr+1}.color      = [get_standard_colors(i) 1];
        config.animation.bodies{bodynr+1}.sides      = 4;
        config.animation.bodies{bodynr+1}.H          = @(y) H_gimbal_inertial(y)*get_Homogenous(eye(3),[0;0;vector_length/2]);
        config.animation.bodies{bodynr+1}.up         = 'z';        
        
        % gimbal axis vector object
        axislength = config.animationsetup.wheel_dimensions(1)*1.5;
        axiswidth  = axislength/30;
        config.animation.bodies{bodynr+2}.dimensions = [axiswidth;axislength;axiswidth];
        config.animation.bodies{bodynr+2}.color      = [[1 1 1]*1 1];
        config.animation.bodies{bodynr+2}.sides      = 4;
        config.animation.bodies{bodynr+2}.H          = H_gimbal_inertial;
        config.animation.bodies{bodynr+2}.up         = 'z';                
     end   
     
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Display selected configuration
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    name_controller    = strrep(func2str(config.functions.gimbal_rate_command),  'get_gamma_dot_c_','');
    name_Mref          = strrep(func2str(config.functions.get_Mref           ),  'get_Mref_',        '');
    
    dir_string   = regexprep(num2str(config.controller.target_direction'),'\s+','_');
    dist_string  = regexprep(num2str(config.model.M_e_disturbance'),'\s+','_');
    if(strcmp(name_Mref,'for_constant_tau') || strcmp(name_Mref,'for_constant_M'))
        dirmagstring = num2str(norm(config.controller.constant_output));
    elseif(strcmp(name_Mref,'maneuver'))
        dirmagstring = num2str(config.controller.target_degrees);  
    else
        dirmagstring = 'unknown';
    end
    
    gimanglestring = regexprep(num2str(config.simulation.initialGimbalAnglesDegrees'),'\s+','_');
    config.about = [config.name '-' name_controller '-' name_Mref '-dir_' dir_string '-' dirmagstring '-dist_' dist_string '-gimbalinit_' gimanglestring];
    disp(config.about)     
    
end
