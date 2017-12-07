function labelHandle = legendl(varargin)
    % Wrapper function for the legend command, which automatically sets the
    % interpreter to Latex, instead of Tex.
    labelHandle = legend(varargin);
    labelHandle.Interpreter = 'latex';
end
