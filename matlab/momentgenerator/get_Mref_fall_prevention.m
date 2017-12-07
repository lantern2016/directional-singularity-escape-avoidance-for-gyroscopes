function Mref = get_Mref_fall_prevention(t, y, config)
% Get reference moment for human fall prevention simulation (Berry et al., 2016)   
    Mref = [0;min(config.controller.tauref_max,2800*t);0];
end

