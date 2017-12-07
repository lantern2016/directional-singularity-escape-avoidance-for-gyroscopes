function config = configuration_smallsat_FIG4_DSEA
%Constants and configuration definitions for the satellite

    config.name = 'smallsat';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Model Properties 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Body properties
    config.model.J_B_x = 214;
    config.model.J_B_y = 201;
    config.model.J_B_z = 500;
    
    % Wheel Properties
    config.model.J_W_f = 0.01;
    config.model.J_W_g = 0.01;
    config.model.J_W_h = 0.02;
    
    % Gimbal Properties
    config.model.J_G_f = 0;
    config.model.J_G_g = 0;
    config.model.J_G_h = 0;    
    
    % Gravity model properties (not applicable for satellite)
    config.model.m_B = 0;
    config.model.gravity = 0;
    config.model.pendulumCOM = [0;0;0];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% CMG Layout
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Number of CMGs, e.g. number of pyramid faces
    config.model.nCMGs = 4;    
    config.model.skewAngle = acos(0.6);
    
    % The pyramid operates with 3D moments
    config.model.planar_torques_only = false;
    
    for i = 1:config.model.nCMGs
        % Rotation about pyramid longitudonal axis
        % Spreading CMGs evenly around. For n = 4
        % that makes a pyramid. Large n approach a
        % cone distribution.
        alpha = (2*pi/config.model.nCMGs)*(i-1);
        z     = [0;0;1];
        R     = get_R_axis(z,-alpha);   
        
        % Initial angular momentum vectors for gamma = 0
        config.model.H0(1:3,i) = R*[0;1;0];
        
        % Gimbal axes along the pyramid faces
        config.model.G0(1:3,i) = R*[sin(config.model.skewAngle);0;cos(config.model.skewAngle)];
        
        % The transverse axes, f = g x h
        config.model.F0(1:3,i) = cross(config.model.G0(1:3,i),config.model.H0(1:3,i));    
        
        % location of the centers of the gimbals in the body frame (not
        % relevant for rotational dynamics, but needed for display purposes)               
        config.animationsetup.gimbal_com(1:3,i) = R*[0.2;0;0]; 
    end
           
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Simulation Properties/Initial Conditions
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Initial attitude and gimbal rates
    config.simulation.InitialMRP             = axis2MRP([0;0;1], pi/2)*0;
    config.simulation.initialGimbalRates     = zeros(config.model.nCMGs,1);
  
    % If we leave the initial angular velocity undefined, it will
    % automatically be selected to obtain zero overal momentum
    config.simulation.InitialAngularVelocity = nan(3,1);
    
    % Initial gimbal angles (degrees)
    config.simulation.initialGimbalAnglesDegrees    = zeros(config.model.nCMGs,1);


    %Timing
    config.simulation.t_end = 5;
    config.simulation.sampling_time = 0.001;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Controller Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Generic Control Configuration
    config.controller.OMEGA_c = 500;
    config.controller.maxGimbalRate = inf;     
    config.controller.sampling_time = 0.01;

    % Acceleration gains
    config.controller.k_gamma        = 40;
    config.controller.k_OMEGA        = 1e-5;  
    
    % Specific Configuration for Wie Controller
    config.controller.k_epsilon0     = 0.01;
    config.controller.k_lambda_1     = 0.01;
    config.controller.k_lambda_2     = 10  ;
    config.controller.omega_epsilon  = 0.5*pi;
    config.controller.phi_i = ((0:2)'*pi/2);     
    config.controller.W_wie          = [1;1;1;1];    
    
    %%% Specific to DSRTP controller

    %%% Singularity measure parameters
    config.controller.sigma_acc = 0.75;
    config.controller.sigma_min = 0.25;
    config.controller.beta      = 10;          
    
    % Perturbation parameters
    config.controller.ktau = 0.5;
    config.controller.d0 = 3;
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Moment Generator Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    % Moment limits
    config.controller.tauref_max   = 10;
    config.controller.tauref_min   = 0.001; 

    % Target direction
    config.controller.target_direction = [1;1;0]; % Reference moment along XY axis
    
    % Control parameters (in case of maneuver simulation)
    config.controller.target_degrees   = 50; % Rotate along previously specified axis, by this amount of degrees

    config.controller.kp = 7; % Proportional control
    config.controller.kv = 3; % Derivative control   
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Select control functions
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    %%%
    %%% Moment generator (choose one)
    %%%
%     config.functions.get_Mref  = @get_Mref_maneuver;
%     config.functions.get_Mref  = @get_Mref_fall_prevention;
%     config.functions.get_Mref  = @get_Mref_none;
%     config.functions.get_Mref  = @get_Mref_disturbance_rejection;    
%     config.functions.get_Mref  = @get_Mref_for_constant_tau;
    config.functions.get_Mref  = @get_Mref_for_constant_M;
    
    % Environment
    config.model.M_e_disturbance = [0;0;0];

    %%%
    %%% Gimbal rate steering law (choose one)
    %%%
%     config.functions.gimbal_rate_command = @get_gamma_dot_c_Wie2005;     
    config.functions.gimbal_rate_command = @get_gamma_dot_c_DSEA  ;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Animation Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
    % Color and dimensions of the main body and the wheels
    config.animationsetup.body_dimensions = [1;1;0.2]*0.28;
    config.animationsetup.body_color      = [0 0.45 0.75 1];
    config.animationsetup.body_sides      = 16;
    
    config.animationsetup.wheel_dimensions = [0.1;0.1;0.02 ];
    config.animationsetup.wheel_color      = [1 0 0 1];
    config.animationsetup.wheel_sides      = 16;   
    
    % Axis limits
    lim = 0.3;
    config.animation.plotsettings.axisLimits = {[-lim lim],[-lim lim],[-lim lim]};
    
    % Playback speed
    config.animation.plotsettings.playBackSpeed = 0.5*2;    
  
end
