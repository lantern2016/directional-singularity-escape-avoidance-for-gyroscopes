function plotHandle = plotplus(varargin)
% Same functionality as plot, but with additional arguments to simplify
% coding. Usage:
%   plotplus(normal plot arguments go here, labels, legends)

    % Number of extra inputs we added to the plot
    nExtraArguments = 3;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%
    %%%  Processing additional inputs
    %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    % Number of normal arguments
    nNormalArguments = nargin-nExtraArguments;    
    
    % Extract the extra arguments
    labels    = varargin{nNormalArguments + 1};
    legends   = varargin{nNormalArguments + 2};
    discrete  = varargin{nNormalArguments + 3};
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%
    %%%  Drawing the normal plot, with normal arguments
    %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    % Make the plot with the normal input, either
    % discrete (stairs) or smooth (plot)
    if(strcmp(discrete,'dt'))
        % For stairs, time samples MUST be rows.
        varargin{2} = varargin{2}';
        plotHandle = stairs(varargin{1:nNormalArguments});
    else
        plotHandle = plot(varargin{1:nNormalArguments});
    end

    % Number of lines in the plot
    nLines = length(plotHandle);    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%
    %%%  Adding Labels
    %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
   
    % Number of labels
    nlabels = length(labels);
    
    % Tex interpreter
    interpreter = 'latex';
   
    % Add an xlabel, unless none is specified
    if(nlabels > 0)     
        if(~isempty(labels{1}))
            h = xlabel(labels{1});
            h.Interpreter = interpreter;
        end
    end
    
    % Add an ylabel, unless none is specified
    if(nlabels > 1)    
        if(~isempty(labels{2}))
            h = ylabel(labels{2});
            h.Interpreter = interpreter;        
        end    
    end

    % Add an zlabel, unless none is specified
    if(nlabels > 2)
        if(~isempty(labels{3}))
            h = zlabel(labels{3});
            h.Interpreter = interpreter;      
        end 
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%
    %%%  Adding the legend
    %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
    
    % Check for presence of legend
    if(~isempty(legends))  
        
        % If there is supposedly only one legend, but if
        % it contains the word ###, use that legend for all
        % plots while substituting ### for the plot number.
        if(length(legends) == 1)
            if(strfind(legends{1},'###'))
                originalLegend = legends{1};

                for k = 1:nLines
                    legends{k} = strrep(originalLegend,'###',num2str(k));
                end
        % Likewise, if the word XYZ, use that legend for all
        % plots while substituting XYZ for x, y, z, respectively.            
            elseif(strfind(legends{1},'XYZ'))
                originalLegend = legends{1};

                for k = 1:nLines
                    legends{k} = strrep(originalLegend,'XYZ',char('w' + k));
                end
            end            
        end
    
    
        % Draw the legend
        h = legend(legends);
        h.Interpreter = interpreter;     
    end
    
end

