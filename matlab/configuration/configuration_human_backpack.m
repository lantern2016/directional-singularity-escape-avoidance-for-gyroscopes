function config = configuration_human_backpack
%Constants and configuration definitions for the human backpack in Berry et al., 2016

    config.name = 'human_backpack';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Model Properties 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Gravity model properties (not applicable for satellite, so not listed in paper)
    config.model.m_B = 70;                 % Main body mass
    config.model.gravity = 9.81;           % Gravity magnitude
    config.model.pendulumCOM = [0;0;0.85]; % Center of mass relative to pivot, in body coordinates    
        
    
    % Body properties
    config.model.J_B_x = 69 + config.model.m_B*config.model.pendulumCOM(3)^2;
    config.model.J_B_y = 69 + config.model.m_B*config.model.pendulumCOM(3)^2;
    config.model.J_B_z = 3.15 ;
    
    % Wheel Properties
    config.model.J_W_f = 1.8e-3;
    config.model.J_W_g = 1.8e-3;
    config.model.J_W_h = 3.6e-3;
    
    % Gimbal Properties
    config.model.J_G_f = 0;
    config.model.J_G_g = 0;
    config.model.J_G_h = 0;       

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% CMG Layout
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Number of CMGs
    config.model.nCMGs = 2;    
    
    % Because this a planar application:
    config.model.planar_torques_only = true;
    
    % we will assume that all gimbal axes are along the +[0;0;1] axis. 
    % In other words, the body frame must be chosen such that this is the case:
    config.model.G0 = repmat([0;0;1],[1 config.model.nCMGs]);    
    
    % Give the initial positions of the angular momentum axes:
    for i = 1:config.model.nCMGs 
        % Initial angular momentum vectors for gamma = 0
        config.model.H0(1:3,i) = [-1;0;0];
        
        % The transverse axes, f = g x h
        config.model.F0(1:3,i) = cross(config.model.G0(1:3,i),config.model.H0(1:3,i));    
        
        % location of the centers of the gimbals in the body frame (not
        % relevant for rotational dynamics, but needed for display purposes)
        gimbal_lateral_pos = 0.25;
        gimbal_lateral_positions = linspace(-gimbal_lateral_pos,gimbal_lateral_pos,config.model.nCMGs);
        config.animationsetup.gimbal_com(1:3,i) = [0;gimbal_lateral_positions(i);0]; 
    end
           
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Simulation Properties/Initial Conditions
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Initial attitude and gimbal rates
    config.simulation.InitialMRP             = axis2MRP([0;1;0], 5/180*pi);
    config.simulation.initialGimbalRates     = zeros(config.model.nCMGs,1);
  
    config.simulation.InitialAngularVelocity = [0;0.56;0]*0;
    
    % Initial gimbal angles (degrees)
    config.simulation.initialGimbalAnglesDegrees    = 90*ones(config.model.nCMGs,1);


    %Timing
    config.simulation.t_end = 0.8;
    config.simulation.sampling_time = 0.001/10*5;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Controller Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Generic Control Configuration
    config.controller.OMEGA_c = 1650;
    config.controller.maxGimbalRate = inf;     
    config.controller.sampling_time = 0.001/10*5;

    % Acceleration gains
    config.controller.k_gamma        = 500*10;
    config.controller.k_OMEGA        = 1e-5;  
    
    % Specific Configuration for Wie Controller
    config.controller.k_epsilon0     = 0.01;
    config.controller.k_lambda_1     = 0.01;
    config.controller.k_lambda_2     = 10  ;
    config.controller.omega_epsilon  = 0.5*pi;
    config.controller.phi_i          = pi/2;     
    config.controller.W_wie          = ones(config.model.nCMGs,1);    
    
    %%% Specific to DSEA controller

    %%% Singularity measure parameters
    config.controller.sigma_acc = 0.7;
    config.controller.sigma_min = 0.25;
    config.controller.beta      = 10;          
    
    % Perturbation parameters
    config.controller.ktau = 0.5;
    config.controller.d0 = 10;
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Moment Generator Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    % Moment limits
    config.controller.tauref_max   = 100;
    config.controller.tauref_min   = 0.1; 

    % Target direction (not applicable for human)
    config.controller.target_direction = nan(3,1); 
    config.controller.target_degrees   = nan;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Select control functions
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    %%%
    %%% Moment generator
    %%%
    config.functions.get_Mref  = @get_Mref_fall_prevention;
    
    % Environment
    config.model.M_e_disturbance = [0;0;0];

    %%%
    %%% Gimbal rate steering law
    %%%
    config.functions.gimbal_rate_command = @get_gamma_dot_c_Wie2005;     
    config.functions.gimbal_rate_command = @get_gamma_dot_c_DSEA  ;
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Animation Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
    % Color and dimensions of the main body and the wheels
    config.animationsetup.body_dimensions = [0.1;0.1;config.model.pendulumCOM(3)*2];
    config.animationsetup.body_color      = [0 0.45 0.75 1];
    config.animationsetup.body_sides      = 4;
    
    config.animationsetup.wheel_dimensions = [0.2;0.2;0.04 ];
    config.animationsetup.wheel_color      = [1 0 0 1];
    config.animationsetup.wheel_sides      = 16;   
    
    % Axis limits
    config.animation.plotsettings.axisLimits = {[-0.5 1.5],[-0.5 0.5],[0 2]};
    
    % Playback speed
    config.animation.plotsettings.playBackSpeed = 0.1;    
  
end
