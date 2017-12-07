function [returnMRP, shadowed] = shadowMRP(originalMRP)
% Calculates shadow MRP parameters. Returns shadow parameters (and its time
% derivatives) if sigma'*sigma > 1 .Otherwise, it returns the original
% parameters. This ensures that the principle rotation is always less than
% |180| degrees, staying well away from the singularity at 360 degrees.

% The shadowed flag returns 1 when a variable change has been made; it 
% returns 0 if no change has been made. This can be used for debugging,
% purposes such as to detect an excessive number of switches.

    % Original parameters
    sigma     = originalMRP(1:3);
    sigma_dot = originalMRP(4:6);  

    % Transformation matrix associated between sigma_dot and omega (see
    % report for details and derivation).
    N = get_N(sigma);
          
    % Shadow parameters calculation
    sigma_shadow = -sigma/(sigma'*sigma);
    sigma_shadow_dot = 1/(sigma'*sigma)*(2/(sigma'*sigma)/(1+sigma'*sigma)*(sigma*sigma')*N'-eye(3))*sigma_dot;

    % Return either the shadow parameters or the original one, such that the
    % returned parameters describe a principle rotation less than |180|
    % degrees
    if(sigma'*sigma > 1)
        returnMRP = [sigma_shadow; sigma_shadow_dot];
        shadowed  = 1;
    else
        returnMRP = originalMRP;
        shadowed  = 0;
    end
end

