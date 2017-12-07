function labelHandle = xlabell(varargin)
    % Wrapper function for the ylabel command, which automatically sets the
    % interpreter to Latex, instead of Tex.
    labelHandle = xlabel(varargin);
    labelHandle.Interpreter = 'latex';
end
