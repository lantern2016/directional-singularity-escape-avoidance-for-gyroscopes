function output = get_standard_colors(color_index)
% Get the default plot colors used by matlab.

   % Color resource: https://nl.mathworks.com/help/matlab/graphics_transition/why-are-plot-lines-different-colors.html
   colors =        [0         0.4470    0.7410
                    0.8500    0.3250    0.0980
                    0.9290    0.6940    0.1250
                    0.4940    0.1840    0.5560
                    0.4660    0.6740    0.1880
                    0.3010    0.7450    0.9330
                    0.6350    0.0780    0.1840];   
                
   % Colors by name index for easy reference
   defaultcolors.blue   = colors(1,:); 
   defaultcolors.orange = colors(2,:); 
   defaultcolors.yellow = colors(3,:); 
   defaultcolors.purple = colors(4,:); 
   defaultcolors.green  = colors(5,:); 
   defaultcolors.cyan   = colors(6,:); 
   defaultcolors.reddish= colors(7,:);
   
   % Number of available colors
   nColors = length(fieldnames(defaultcolors));
   
   % If index is supplied, return that color
   if(nargin == 1)
       % Start counting from start if color number is not available using
       % the modulo operator
       color_index_fixed = mod(color_index-1,nColors)+1;
       output = colors(color_index_fixed,:);
   else
       % If no index is supplied, return the entire color structure with
       % name indexing
       output = defaultcolors;
   end
   
end