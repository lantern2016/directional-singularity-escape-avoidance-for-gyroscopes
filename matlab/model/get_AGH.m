function [A, G, H] = get_AGH(y, config)
% Return the gimbal frame axes given their initial configuration and the
% current gimbal axes

    % Full matrices for 3D applications
    gamma     = y(config.model.id.gamma);
    G = config.model.G0;
    H =  config.model.H0*diag(cos(gamma)) + config.model.F0*diag(sin(gamma));
    A = -config.model.H0*diag(sin(gamma)) + config.model.F0*diag(cos(gamma));
end