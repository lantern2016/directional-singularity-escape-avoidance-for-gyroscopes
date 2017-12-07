function fig = figurehalf(location)
% Create and return and empty figure that fills the right half of the
% screen. Then, when you keep MATLAB on the left, there is no more need to
% move figures around.

    % Create the figure window with these specs. All values are percentages
    % relative to your own screen, so it works on screens with different
    % sizes.
    
    % Location of the leftmost side of the window, relative to the left
    % side of your screen.
    
    if(nargin == 1 && strcmp(location,'left'))
        StartX = 0.0;
    else
        StartX = 0.52;
    end    
    
    % Location of the bottom side of the window, relative to the bottom
    % side of your screen.    
    StartY = 0.0; 
    
    % Height, width (again as a percentage of your screen)
    Width = 0.48;
    Height = 0.9;   
    
    % Create the figure with the specified measures
    fig = figure('units','normalized','position',[StartX StartY Width Height]);
    
    % In newer matlab versions, turn off auto update of legends    
    try set(fig,'defaultLegendAutoUpdate','off'); catch ; end;    
    
    % For convenience, also turn on hold.
    hold on;

end