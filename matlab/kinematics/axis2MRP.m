function sigma = axis2MRP(axes, angle)
% Convert euler eigen axis and angle to the corresponding MRP

    % Allow this function to accept an array of axes and angles.
    nSteps = length(angle);
    sigma = zeros(3,nSteps);
    
    % Convert each axis/angle pair to an MRP
    for i = 1:nSteps
        % Convert axis to unit vector
        axis = get_unit_vector(axes(:,i));

        % Basic definition of MRP from axis and angle
        sigma_unmasked = axis*tan(angle(i)/4);

        % Apply shadow mapping if necessary.
        [returnMRP, ~] = shadowMRP([sigma_unmasked;0;0;0]);
        sigma(:,i) = returnMRP(1:3);        
    end


end

