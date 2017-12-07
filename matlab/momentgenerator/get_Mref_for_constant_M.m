function Mref = get_Mref_for_constant_M(t, y, config)
% Get a constant internal moment
        
    % Reference moment to be produced by gimbal rotation
    Mref = config.controller.constant_output;
   
end
