clc
clear all
close all
warning('This script is included strictly for transparency of the results in the paper.')
warning('For easier interpretation of the results, please simply run "simulator.m"')

%%%
%%% Load simulation results
%%%
DSEA = 1;
ODSR = 2;
results(DSEA) = load('results/smallsat-DSEA-disturbance_rejection-dir_NaN_NaN_NaN-unknown-dist_0.5_0_0-gimbalinit_0_0_0_0.mat');
results(ODSR) = load('results/smallsat-Wie2005-disturbance_rejection-dir_NaN_NaN_NaN-unknown-dist_0.5_0_0-gimbalinit_0_0_0_0.mat');


%%%
%%% Create canvas and colors
%%%
pFig = figurehalf('right');
nRows = 4;
nCols = 3;
black = [1 1 1]*0;
grey  = [1 1 1]*0.6;

%%%
%%% Function to get maximum peak in data. Used to choose vertical limits
%%% of a plot, rounded to the desired number of decimals.
%%%
roundmaxval = @(data, decimals) ceil(max(max(abs(data)))*10^decimals)/10^decimals;

%%%
%%% Create plots for both DSEA and O-DSR
%%%
for controller = DSEA:ODSR
    
    %%%
    %%% Load generic results for easier plotting
    %%%
    time    = results(controller).time;
    delta   = results(controller).delta;
    controlsignals = results(controller).control;
    [sigma, sigma_dot, omega, gamma, gamma_dot, OMEGA] =  get_states(results(controller).Y, results(controller).config);
    gamma_dot_e_Vspace = [controlsignals.gamma_dot_e_Vspace];
    psi = [delta.psi];
    trange = [0 time(end)];
    MRPref = repmat(zeros(3,1),[1 length(time)]);
    MRPErr = MRPref - sigma;
    MRPErrnorm = sqrt(sum(MRPErr.^2));
    MRPErrAngle = 4*atan(MRPErrnorm)*180/pi;
    
    %%%
    %%% Plot of moments
    %%%     
    subplotplus(nRows,nCols,1+(controller-1)*2,1);      
    plot(time, results(controller).M_internal(1,:),    'Color', black, 'LineStyle', '-'); hold on;       
    plot(time, results(controller).M_internal(2,:),    'Color', black, 'LineStyle', '--'); hold on;      
    plot(time, results(controller).M_internal(3,:),    'Color', grey,  'LineStyle', '-'); hold on;           
    plot(time, results(controller).Mref(1,:),    'Color', black, 'LineStyle', ':'); hold on;             
    xlim(trange);
    legendl('$x$', '$y$', '$z$','$x$\reftext');
    ylabell('$\bMint \unitb{Nm}$');    
    if(controller == DSEA)
        ylim([-0.01 0.1]*results(controller).config.controller.tauref_max)
        reflabel('a')
    else
        reflabel('g')
        ylim([-0.2 1.7]*results(controller).config.controller.tauref_max)
    end     

    %%%
    %%% Plot of singularity measures
    %%%    
    subplotplus(nRows,nCols,1+(controller-1)*2,2);       
    plot(time, [delta.RA]     ,    'Color', black, 'LineStyle', '-'); hold on;       
    plot(time, [delta.man]    ,    'Color', black, 'LineStyle', '--'); hold on;      
    plot(time, [delta.p_i_bar],    'Color', grey,  'LineStyle', '-'); hold on;          
    xlim(trange);
    ylim([0 1.05])
    ylabell('$\delta \unitb{-}$');
    legendl('$\deltaTrack$', '$\deltaMAN$', '$\deltasat$');    
    if(controller == DSEA)
        reflabel('b')
        title('DSEA','FontWeight','normal');
    else
        reflabel('h')
        title('o-DSR','FontWeight','normal')
    end   
    
    %%%
    %%% Plot of moment error and MRP error scalar
    %%%    
    subplotplus(nRows,nCols,2+(controller-1)*2,1);      
    plot(time,MRPErrAngle, 'Color', black); 
    ylabell('$\onenorm{\bro} \unitb{deg}$');      
    xlim(trange);
    if(controller == DSEA)
        reflabel('d')
        xlabell('$t\unitb{s}$');
    else
        reflabel('j')
        xlabell('$t\unitb{s}$');
    end

        
    %%%
    %%% Plot of actual gimbal rates
    %%%       
    subplotplus(nRows,nCols,1+(controller-1)*2,3);
    plot(time,gamma_dot(1,:), 'Color', black, 'LineStyle', '--');hold on;
    plot(time,gamma_dot(2,:), 'Color', grey , 'LineStyle', '--');hold on;
    plot(time,gamma_dot(3,:), 'Color', black, 'LineStyle', '-' );hold on;
    plot(time,gamma_dot(4,:), 'Color', grey , 'LineStyle', '-' );hold on;        
    xlim(trange);
    ylabell('$\dgamma_i \unitb{rad/s}$');
    legendl('1','2','3','4');    
    if(controller == DSEA)
        ylim([-1 1]*0.4)
        reflabel('c')
    else
        ylim([-1 1]*1)
        reflabel('i')
    end
   
    %%%
    %%% Plot of escape rates in V space
    %%%     
    subplotplus(nRows,nCols,2+(controller-1)*2,2);
    plot(time,gamma_dot_e_Vspace(1,:), 'Color', black, 'LineStyle', '--');hold on;
    plot(time,gamma_dot_e_Vspace(2,:), 'Color', grey , 'LineStyle', '--');hold on;
    plot(time,gamma_dot_e_Vspace(3,:), 'Color', black, 'LineStyle', '-' );hold on;
    plot(time,gamma_dot_e_Vspace(4,:), 'Color', grey , 'LineStyle', '-' );hold on;    
    if(controller == DSEA)
        reflabel('e')
        xlabell('$t\unitb{s}$');
    else
        reflabel('k')
        xlabell('$t\unitb{s}$');
    end 
    ylabell('$\bvjT \bdgammae \unitb{rad/s}$');
    legendl('1','2','3','4');      
    ylim([-1 1]*roundmaxval([results(controller).control.gamma_dot_e_Vspace],2))
    xlim(trange);
    
    %%%
    %%% Plot of gimbal angles
    %%%     
    subplotplus(nRows,nCols,2+(controller-1)*2,3);
    [wrappedtime, wrappedangle] = wrapangle(time,gamma(1,:));
    plot(wrappedtime,wrappedangle, 'Color', black, 'LineStyle', '--');hold on;
    [wrappedtime, wrappedangle] = wrapangle(time,gamma(2,:));
    plot(wrappedtime,wrappedangle, 'Color', grey , 'LineStyle', '--');hold on;
    [wrappedtime, wrappedangle] = wrapangle(time,gamma(3,:));
    plot(wrappedtime,wrappedangle, 'Color', black, 'LineStyle', '-' );hold on;
    [wrappedtime, wrappedangle] = wrapangle(time,gamma(4,:));
    plot(wrappedtime,wrappedangle, 'Color', grey , 'LineStyle', '-' );hold on;    
    ylim([-1 1]*pi)
    xlim(trange);
    ylabell('$\gamma_i \unitb{rad}$');
    legendl('1','2','3','4');
    xlabell('$t\unitb{s}$');    
    set(gca,'YTick',-pi:pi/2:pi);set(gca,'YTickLabel',{'-\pi','','0','','\pi'})
    if(controller == DSEA)
        reflabel('f')
    else
        reflabel('l')
        xlabell('$t\unitb{s}$');
    end    
end


savestring = [results(DSEA).config.about '___-----___' results(ODSR).config.about];
SaveLatexPlot(pFig, 1.05, 0.6, savestring)
disp(savestring)
      
