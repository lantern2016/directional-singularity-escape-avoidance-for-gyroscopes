function [fig] = figurefull
% Create and return and empty figure that fills the whole screen
   
    % Create the figure with the specified measures
    fig = figure('units','normalized','position',[0 0 1 1]);
    
    % In newer matlab versions, turn off auto update of legends    
    try set(fig,'defaultLegendAutoUpdate','off'); catch ; end;

    % For convenience, also turn on hold.
    hold on;

end