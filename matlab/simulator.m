%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%
%%% This code accompanies the paper:
%%%
%%% Valk, L., Berry, A., and Vallery, H.,
%%% "Directional Singularity Escape and Avoidance for Single-Gimbal Control Moment Gyroscopes,"
%%% Journal of Guidance, Control, and Dynamics
%%% http://dx.doi.org/10.2514/1.G003132
%%%
%%% Code (c) 2017-2018 Laurens Valk (laurensvalk@gmail.com)
%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Prepare environment
clc;
clear all;
addpath('configuration','kinematics','graphics','model','cmgsteering','momentgenerator','makeplots');
warning(fileread('../README.md'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% System model selection
%%% Choose any one of the following settings files
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Human model (Berry et al., 2016)
% SystemModel  = @configuration_human_backpack;

% Satellite model with pyramid CMG (Valk et al., 2018)
SystemModel  = @configuration_smallsat;    

% Satellite model as above, but with the precise initial conditions and
% settings corresponding to the figures in (Valk et al., 2018), for
% transparency. Uncomment one configuration at a time to run. These result
% in basic plots and the results are saved as MAT files. Running the
% supplementary scripts in the "makeplots" folder renders the
% black-and-white figures as in the paper, with the same zoom levels, etc.
%
% For most usecases, however, just modify the example model above instead.
%
% SystemModel  = @configuration_smallsat_FIG4_DSEA;
% SystemModel  = @configuration_smallsat_FIG5_DSEA;
% SystemModel  = @configuration_smallsat_FIG5_ODSR;
% SystemModel  = @configuration_smallsat_FIG6_DSEA;
% SystemModel  = @configuration_smallsat_FIG6_ODSR;
% SystemModel  = @configuration_smallsat_FIG7_DSEA;
% SystemModel  = @configuration_smallsat_FIG7_ODSR;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%
%%% Simulation: Integration of ODE
%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set up constants and the selected functions
config = SystemModel();

% Autmatically derive additional settings from the ones defined above. 
config = configuration_generic(config); 

disp('...................'); disp('Starting Simulation');

% Time vector
h = config.simulation.sampling_time;
time = 0:h:config.simulation.t_end;
nSteps = length(time);

% Zero state matrix
Y          = zeros(config.model.nStates,nSteps);
Y(:,1)     = config.simulation.y0;
M_internal = zeros(3, nSteps);
tau        = zeros(3, nSteps);
% Zero control matrices
Mref                  = zeros(3, nSteps);
tauref                = zeros(3, nSteps);
U                    = zeros(config.model.nCMGs*2,nSteps);
for k=1:nSteps; delta(k)   = get_singularity_measures     ([],[],[],config); end
for k=1:nSteps; control(k) = config.functions.gimbal_rate_command([],[],[],config); end

% Main simulation loop
for k=1:nSteps
    %Read the current state
    ynow = Y(:,k);
    tnow = time(k);

    % Determine if the rate controller must be evaluated 
    % at the current simulation time step.
    if(mod(k-1,config.simulation.nSimStepsPerControlSample) == 0)
        % Get the reference moment
        Mref(:,k)   = config.functions.get_Mref(tnow, ynow, config);         % Get the internal reference moment
        tauref(:,k) = get_tauref_from_Mref(Mref(:,k), ynow, config);  % Get gyroscopic reference moment

        % Get the corresponding gimbal rates using the rate controller
        control(k) = config.functions.gimbal_rate_command(tnow, ynow, tauref(:,k), config);               
    else
        % If the control signal is not renewed, it is equal to the previous value.
        Mref(:,k) =Mref(:,k-1); tauref(:,k) = tauref(:,k-1); delta(k) = delta(k-1);
        control(k) = control(k-1);
    end

    % Store the singularity measures
    delta(k) = get_singularity_measures(tnow, ynow, tauref(:,k), config);

    % The motor controllers are assumed to be continous
    U(:,k) = get_u(tnow, ynow, control(k).gamma_dot_c_bounded, config);    

    %Perform the Runge-Kutta-4 integration scheme
    t1 = tnow      ; y1 = ynow;           [k1, M_internal(:,k), tau(:,k)] = get_y_dot(t1, y1, get_u(tnow, y1, control(k).gamma_dot_c_bounded, config), config);
    t2 = tnow + h/2; y2 = ynow+h*0.5*k1;  [k2                           ] = get_y_dot(t2, y2, get_u(tnow, y2, control(k).gamma_dot_c_bounded, config), config);
    t3 = tnow + h/2; y3 = ynow+h*0.5*k2;  [k3                           ] = get_y_dot(t3, y3, get_u(tnow, y3, control(k).gamma_dot_c_bounded, config), config);
    t4 = tnow + h  ; y4 = ynow+h*k3;      [k4                           ] = get_y_dot(t4, y4, get_u(tnow, y4, control(k).gamma_dot_c_bounded, config), config);   
    ynext = ynow + h/6*(k1+2*k2+2*k3+k4);

    % Perform shadow paramater operation if necessary
    [ynext(config.model.id.MRPs), ~] = shadowMRP(ynext(config.model.id.MRPs));

    %Store the next state, except if we're at the final time.
    if(k < length(time))
        Y(:,k+1) = ynext;
    end

end

% Save simulation and configuration data for reproduction of results
save(['results/' config.about '.mat'],'config','time', 'Mref', 'M_internal', 'tauref','tau', 'Y','U', 'control', 'delta')

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%
%%% Plotting Results
%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Extract state variables for easy plotting
[sigma, sigma_dot, omega, gamma, gamma_dot, OMEGA] =  get_states(Y, config);

hPlot = figurefull;
nCols = 3;
nRows = 3;

%%% Plotting Moments
subplotplus(nRows,nCols,1,1);
hLines1 = plotplus(time,M_internal,    {[],'$M$ (Nm)'},{'(XYZ)'},'ct'); hold on   
hLines2 = plotplus(time,Mref, '--',  [],[], 'dt');hold on 
samecolors(hLines1, hLines2)

%%% Plotting Angular velocity
subplotplus(nRows,nCols,2,1);
plotplus(time,omega*180/pi,          {[],'$\omega$ (deg/s)'},{'(XYZ)'}, 'ct');

%%% Plotting Gimbal Angles
subplotplus(nRows,nCols,3,2);
plotplus(time,gamma*180/pi,          {[],'$\gamma$ (deg)'},{'(###)'}, 'ct');    

%%% Plotting Singularity Measures
subplotplus(nRows,nCols,1,2);
delta_res = [[delta.RA];
             [delta.man];   
             [delta.p_i_bar]];
plotplus(time,delta_res,  {[],'$\delta$ (-)'},{'$\delta_{track}$','$\delta_{man}$','$\delta_{pot}$'},'ct');         
ylim([-0.05 1.05])

%%% Plotting di, pi
subplotplus(nRows,nCols,2,2); 
hLines1 = plotplus(time,[delta.p_i], '-', {[],[]},{'(###)'}, 'dt');hold on;
hLines2 = plotplus(time,[delta.d_i], '--', {[],'$p_i$ [-], $d_i$ [-\,-]'},{}, 'dt');

samecolors(hLines1, hLines2)
ylim([-0.05 1.05])

%%% Plotting gamma speed
subplotplus(nRows,nCols,1,3);
hLines1 = plotplus(time,gamma_dot,       {'$t$','$\dot{\gamma}$ [-], $\dot{\gamma}_c$ [-\,-] (rad/s)'},{'(###)'},'ct'); hold on
hLines2 = plotplus(time,[control.gamma_dot_c] , '--', [],[], 'dt');
samecolors(hLines1, hLines2)
subplotplus(nRows,nCols,2,3);
hLines1 = plotplus(time,[control.gamma_dot_e],       {'$t$','$\dot{\gamma}_e$ [-], $\dot{\gamma}_s$ [-\,-] (rad/s)'},{'(###)'}, 'dt'); hold on
hLines2 = plotplus(time,[control.gamma_dot_d] , '--', [],[], 'dt');
samecolors(hLines1, hLines2)
subplotplus(nRows,nCols,3,3);
hLines1 = plotplus(time,[control.gamma_dot_e_Vspace],       {'$t$','$V^T \dot{\gamma}_e$ (rad/s)'},{'(###)'}, 'dt'); hold on

%%% Plotting Atitude MRPs
subplotplus(nRows,nCols,3,1);
plotplus(time,sigma,       {'$t$','$\rho$ (-)'},{'(###)'},'ct'); hold on

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%
%%% Running the Animation
%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Animate(time,Y, config.animation.bodies, config.animation.plotsettings);
