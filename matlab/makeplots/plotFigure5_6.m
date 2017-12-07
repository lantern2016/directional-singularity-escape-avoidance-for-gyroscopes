clc
clear all
close all
warning('This script is included strictly for transparency of the results in the paper.')
warning('For easier interpretation of the results, please simply run "simulator.m"')

%%%
%%% Choose which plot to make (Either 5 or 6)
%%%
plotFigure = 5;

%%%
%%% Load simulation results
%%%
DSEA = 1;
ODSR = 2;

try
    if(plotFigure == 5)
        % Load the results for Figure 5
        results(DSEA) = load('results/smallsat-DSEA-for_constant_M-dir_1_0_0-10-dist_0_0_0-gimbalinit_0_0_0_0.mat');
        results(ODSR) = load('results/smallsat-Wie2005-for_constant_M-dir_1_0_0-10-dist_0_0_0-gimbalinit_0_0_0_0.mat');
    elseif(plotFigure == 6)
        % Load the results for Figure 6   
        results(DSEA) = load('results/smallsat-DSEA-for_constant_M-dir_-1_0_0-10-dist_0_0_0-gimbalinit_-90_180_90_0.mat');
        results(ODSR) = load('results/smallsat-Wie2005-for_constant_M-dir_-1_0_0-10-dist_0_0_0-gimbalinit_-90_180_90_0.mat');
    else
        error('Select either plot 5 or 6')
        return
    end
catch
    error('First run "simulator.m" to generate the appropriate MAT files.')
end
    
%%%
%%% Create canvas and colors
%%%

pFig = figurehalf('right');
nRows = 3;
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
    gamma_dot_d = [controlsignals.gamma_dot_d];
    psi = [delta.psi];
    trange = [0 time(end)];
    
              
    %%%
    %%% Plot of moments
    %%%     
    subplotplus(nRows,nCols,1,2*controller-1);      
    plot(time, results(controller).M_internal(1,:), 'Color', black, 'LineStyle', '-'); hold on;       
    plot(time, results(controller).M_internal(2,:), 'Color', black, 'LineStyle', '--'); hold on;      
    plot(time, results(controller).M_internal(3,:), 'Color', grey,  'LineStyle', '-'); hold on;           
    plot(time, results(controller).Mref(1,:),       'Color', black, 'LineStyle', ':'); hold on;             
    %%% Add title to the plot
    if(controller == DSEA)
        title('DSEA','FontWeight','normal');
    else
        title('O-DSR','FontWeight','normal')
    end     
    %%% Choose the vertical limit, depending on whether we plot Figure 5 or 6
    if plotFigure == 5
        ylim([-0.4 1.2]*results(controller).config.controller.tauref_max)
    else
        ylim([-1.5 0.2]*results(controller).config.controller.tauref_max)
    end
    legendl('$x$', '$y$', '$z$','$x$\reftext');
    ylabell('$\bMint \unitb{Nm}$');    
    xlim(trange);
    % Add subfigure number
    if(controller == DSEA)
        reflabel('a')
    else
        reflabel('g')
    end   
   
    %%%
    %%% Plot of singularity measures
    %%%    
    subplotplus(nRows,nCols,2,2*controller-1);       
    plot(time, [delta.RA]     ,    'Color', black, 'LineStyle', '-'); hold on;       
    plot(time, [delta.man]    ,    'Color', black, 'LineStyle', '--'); hold on;      
    plot(time, [delta.p_i_bar],    'Color', grey,  'LineStyle', '-'); hold on;          
    xlim(trange);
    ylim([0 1.05])
    ylabell('$\delta \unitb{-}$');
    legendl('$\deltaTrack$', '$\deltaMAN$', '$\deltasat$');    
    % Add subfigure number
    if(controller == DSEA)
        reflabel('b')
    else
        reflabel('h')
    end     
        
    %%%
    %%% Plot of gimbal rates
    %%%       
    subplotplus(nRows,nCols,3,2*controller-1);
    plot(time,gamma_dot(1,:), 'Color', black, 'LineStyle', '--');hold on;
    plot(time,gamma_dot(2,:), 'Color', grey , 'LineStyle', '--');hold on;
    plot(time,gamma_dot(3,:), 'Color', black, 'LineStyle', '-' );hold on;
    plot(time,gamma_dot(4,:), 'Color', grey , 'LineStyle', '-' );hold on;        
    ylim([-1 1]*4)
    xlim(trange);
    ylabell('$\dgamma_i \unitb{rad/s}$');
    legendl('1','2','3','4');
    xlabell('$t\unitb{s}$');    
    % Add subfigure number
    if(controller == DSEA)
        reflabel('c')
    else
        reflabel('i')
        xlabell('$t\unitb{s}$');
    end
    
    %%%
    %%% Draw the plots that apply only to the DSEA Controller
    %%%
    if controller == DSEA
        %%%
        %%% Plot of secondary task rates
        %%%    
        subplotplus(nRows,nCols,1,2);
        plot(time,gamma_dot_d(1,:), 'Color', black, 'LineStyle', '--');hold on;
        plot(time,gamma_dot_d(2,:), 'Color', grey , 'LineStyle', '--');hold on;
        plot(time,gamma_dot_d(3,:), 'Color', black, 'LineStyle', '-' );hold on;
        plot(time,gamma_dot_d(4,:), 'Color', grey , 'LineStyle', '-' );hold on;    
        ylabell('$\dgammai\unitb{rad/s}$');
        legendl('1','2','3','4');
        reflabel('d')  
        ylim([-1 1]*roundmaxval([results(DSEA).control.gamma_dot_d],2))
        xlim(trange);        

        %%%
        %%% Plot of escape rates in V space
        %%%     
        subplotplus(nRows,nCols,2,2);
        plot(time,gamma_dot_e_Vspace(1,:), 'Color', black, 'LineStyle', '--');hold on;
        plot(time,gamma_dot_e_Vspace(2,:), 'Color', grey , 'LineStyle', '--');hold on;
        plot(time,gamma_dot_e_Vspace(3,:), 'Color', black, 'LineStyle', '-' );hold on;
        plot(time,gamma_dot_e_Vspace(4,:), 'Color', grey , 'LineStyle', '-' );hold on;    
        ylabell('$\bvjT \bdgammae \unitb{rad/s}$');
        legendl('1','2','3','4');
        reflabel('e')  
        ylim([-1 1]*roundmaxval([results(DSEA).control.gamma_dot_e_Vspace],2))
        xlim(trange);

        %%%
        %%% Plot of gimbal angles
        %%%     
        subplotplus(nRows,nCols,3,2);
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
        set(gca,'YTick',-pi:pi/2:pi);set(gca,'YTickLabel',{'-\pi','','0','','\pi'})
        ylabell('$\gamma_i \unitb{rad}$');
        legendl('1','2','3','4');
        reflabel('f')
        xlabell('$t\unitb{s}$');   
    end
    
end


        
%%%
%%% Save the plot in Tex format
%%%

savestring = [results(1).config.about '___-----___' results(2).config.about];
SaveLatexPlot(pFig, 1.05, 0.47, savestring)
disp(savestring)
      
