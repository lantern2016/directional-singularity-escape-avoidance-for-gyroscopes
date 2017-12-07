%%%
%%% Load simulation results
%%%
clc
close all
clear all

warning('This script is included strictly for transparency of the results in the paper.')
warning('For easier interpretation of the results, please simply run "simulator.m"')

try
    results = load('results/smallsat-DSEA-for_constant_M-dir_1_1_0-10-dist_0_0_0-gimbalinit_0_0_0_0.mat');
catch
    error('First run "simulator.m" to generate the appropriate MAT file.')
end

%%%
%%% Create canvas and colors
%%%
pFig = figurehalf('right');
nRows = 1;
nCols = 2;
black = [1 1 1]*0;
grey  = [1 1 1]*0.6;

    
%%%
%%% Load generic results for easier plotting
%%%
time    = results.time;
delta   = results.delta;
control = results.control;
[sigma, sigma_dot, omega, gamma, gamma_dot, OMEGA] =  get_states(results.Y, results.config);

%%%
%%% Plot of singularity measures
%%%    
subplotplus(nRows,nCols,1,2);     
plot(time, [delta.RA]     ,    'Color', black, 'LineStyle', '-'); hold on;       
plot(time, [delta.man]    ,    'Color', black, 'LineStyle', '--'); hold on;      
plot(time, [delta.p_i_bar],    'Color', grey,  'LineStyle', '-'); hold on;   
xlim([0 5]);
ylim([0 1.05])
xlabell('$t \unitb{s}$');
ylabell('$\delta\unitb{-}$');
legendl('$\deltaTrack$', '$\deltaMAN$', '$\deltasat$');
reflabel('b')

%%%
%%% Plot of moments
%%%    
subplotplus(nRows,nCols,1,1);      
plot(time, results.M_internal(1,:),    'Color', grey, 'LineStyle', '-'); hold on;       
plot(time, results.M_internal(2,:),    'Color', black, 'LineStyle', '--'); hold on;      
plot(time, results.M_internal(3,:),    'Color', black,  'LineStyle', '-'); hold on;           
plot(time, results.Mref(1,:),    'Color', black, 'LineStyle', ':'); hold on;             
ll = legendl('$x$', '$y$', '$z$','$x,y$\reftext');
xlim([0 5])
xlabell('$t \unitb{s}$');
ylabell('$\bMint \unitb{Nm}$');
reflabel('a')

ylim([-0.0 1.0]*results.config.controller.tauref_max);

% Save plot in paper format, if Tikz is installed.
SaveLatexPlot(pFig, 0.55, 0.15, 'SingularityMeasures')

      
