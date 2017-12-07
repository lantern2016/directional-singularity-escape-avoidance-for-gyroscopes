function quaternions = MRP2quaternion(MRPs)
% Convert MRPs to corresponding quaternions

    % Allow this function to accept MRPs for multiple time instances
    nSteps      = size(MRPs,2);
    quaternions = zeros(4,nSteps);
    
    % Do the conversion for each time step
    for i = 1:nSteps
        %Current sigma value
        sigma = MRPs(:,i);
        
        %Quaternion
        sigma_norm = sigma'*sigma;
        q4 = (1-sigma_norm)/(1+sigma_norm);
        q123 = sigma*(1+q4);        

        % Stacked as one vector
        quaternions(:,i) = [q123; q4];
    end
end