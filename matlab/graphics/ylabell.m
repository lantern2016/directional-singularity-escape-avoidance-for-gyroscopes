function labelHandle = ylabell(varargin)
    % Wrapper function for the ylabel command, which automatically sets the
    % interpreter to Latex, instead of Tex.
    labelHandle = ylabel(varargin);
    labelHandle.Interpreter = 'latex';
end
