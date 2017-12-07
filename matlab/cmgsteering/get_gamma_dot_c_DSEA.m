function output = get_gamma_dot_c_DSEA(t, y, tau_r, config)
% Get the control input for the current state and desired reference moment,
% using the DSEA method, which is presented in

% Valk, L., Berry, A., and Vallery, H.,
% "Directional Singularity Escape and Avoidance for Single-Gimbal Control Moment Gyroscopes,"
% Journal of Guidance, Control, and Dynamics
% http://dx.doi.org/10.2514/1.G003132

    % A persistent variable across calls to this function that contains the
    % perturbation directions (-1, 0, 1) for each gimbal.
    persistent theta_previous
    persistent dgamma_previous;

    % Initialize output structure
    output.gamma_dot_c         = zeros(config.model.nCMGs,1);
    output.gamma_dot_c_bounded = zeros(config.model.nCMGs,1);
    output.gamma_dot_p         = zeros(config.model.nCMGs,1);
    output.gamma_dot_d         = zeros(config.model.nCMGs,1);
    output.gamma_dot_e         = zeros(config.model.nCMGs,1);
    output.gamma_dot_d_Vspace  = zeros(config.model.nCMGs,1);
    output.gamma_dot_e_Vspace  = zeros(config.model.nCMGs,1);
    output.tau_c_Uspace        = zeros(config.model.outputdimension,1);
    output.tau_d_Uspace        = zeros(config.model.outputdimension,1);
    output.tau_e_Uspace        = zeros(config.model.outputdimension,1);
    output.tau_error_Uspace    = zeros(config.model.outputdimension,1);
    output.tau_c               = zeros(config.model.outputdimension,1);
    output.tau_p               = zeros(config.model.outputdimension,1);
    output.tau_d               = zeros(config.model.outputdimension,1);
    output.tau_e               = zeros(config.model.outputdimension,1);   
    output.tau_error           = zeros(config.model.outputdimension,1);   
    
    % For initialization, only return this 'empty' output structure
    if(isempty(t)) 
        return;
    end  

    % Initialize previous perturbation directions to zero
    if(isempty(theta_previous))
        theta_previous = zeros(config.model.nCMGs,1);
        dgamma_previous= zeros(config.model.nCMGs,1);
    end

    % Number of CMGS
    n = config.model.nCMGs;        
      
    % Singularity measures and system matrices. These have already been
    % adjusted for the 2D and 3D case, depending on the selected
    % configuration.
    [delta, A, G, H, U, S, V] = get_singularity_measures(t, y, tau_r, config);
  
    % Also reduce the reference moment to the appropriate dimension
    tau_r = tau_r(1:config.model.outputdimension);

    % Singular vectors associated with smallest singular value
    Um = U(:,config.model.outputdimension);
    Vm = V(:,config.model.outputdimension); 
    sigmas  = diag(S);
    sigma_m = sigmas(config.model.outputdimension);
    
    
    %%%% 
    %%%% Perturbation strategy
    %%%% 
    
    % Gimbal index of the unsaturated gimbals
    anti_saturated_index = find((H'*tau_r) < 0);

    % Number of unsaturated gimbals
    nAntiSaturated = length(anti_saturated_index); 
    
    % Check if we need perturbation at all:        
    if(norm(tau_r) < config.controller.tauref_min || isempty(anti_saturated_index))
        % The perturbation strategy is relevant only for nonzero reference
        % moments, and when at least one gimbal needs perturbation.
        % Otherwise, the perturbation is simply zero:       
        output.gamma_dot_d = zeros(n,1);
        theta_previous = zeros(config.model.nCMGs,1);
        dgamma_previous= zeros(config.model.nCMGs,1);        
    else     
        %%%%
        %%%% ALGORITHM 1 from paper:
        %%%%
        %%%% Calculating Gimbal Perturbation magnitude properties, assuming the 
        %%%% system is at the RA singularity closest to the current
        %%%% configuration. We will ultimately use this information only to
        %%%% determine a direction for the perturbation. The actual
        %%%% perturbation magnitude will be determined later, based on the
        %%%% actual system configuration.   

        % Initialize the perturbation and A matrix at zero.
        dgamma_magnitude_at_singularity = zeros(n,1);
        dgamma_magnitude           = zeros(n,1);
        A_at_singularity = zeros(size(A));        
        
        
        % For all unsaturated gimbals, determine the f_i axis and the
        % perturbation magnitude. We need to do this only for the
        % unsaturated gimbals because these are the ones we will perturb.
        % Because we look at the perturbation error only, we do not need to
        % find expressions for the other f_i.
        for k = 1:nAntiSaturated
            
            % Find the gimbal number
            i = anti_saturated_index(k);

            % Find the f_i orientation that the gimbal would have when we
            % are at the singularity closest to the current orientation.
            
            % At the singularity, tau_r is maximally projected onto h_i,
            % and we can obtain the orientation of f_i using the diagrams
            % in the paper, which differs slightly for the 2D and 3D case:
            if(config.model.planar_torques_only)
                % Torque axis in the planar case
                A_at_singularity(:,i) = get_unit_vector([tau_r(2); -tau_r(1)]);
            else
                % Torque axis in 3D
                A_at_singularity(:,i) = get_unit_vector(-cross( G(:,i)    , tau_r    ));
            end
    
            % The perturbation magnitude of the unsaturated gimbals is:
            dgamma_magnitude(i)               = config.controller.d0*norm(tau_r/config.controller.tauref_max)^config.controller.ktau*(1/(norm(tau_r)^3+config.controller.tauref_min^3))*min(H(:,i)'*tau_r,0)^2*sqrt(tau_r'*U*(eye(config.model.outputdimension)-diag(delta.z))^2*U'*tau_r);
            
            % Exactly at the singularity, it is:
            dgamma_magnitude_at_singularity(i) = config.controller.d0*norm(tau_r/config.controller.tauref_max)^config.controller.ktau*(norm(tau_r)/(norm(tau_r)^3+config.controller.tauref_min^3))*(tau_r'*tau_r - (G(:,i)'*tau_r)^2);
            
        end
        
        %%%% 
        %%%% Perturbation direction selection.
        %%%%  
        
        %%% First, take all binary combinations of all unsaturated gimbals,
        %%% which are candidates for perturbation. For any permutation
        %%% theta, we do not need to consider the permutation -theta, as it
        %%% would result in exactly the same additional escape error. 
        
        % Optimum cost is initially infinity, until we find a lower cost
        tau_dRAnorm_minimum = inf;
        
        % Initialize optimal directions at zero.          
        theta_tilde = zeros(n,1);
                                    
        % From the above discussion, we need to consider 2^(nUnsaturated-1)
        % options, and check the cost associated with each:
        for k = 1:2^nAntiSaturated  
            % Initialize optimal directions at zero. In the next step, we will
            % make the directions nonzero only for those gimbals that require
            % perturbation (the unsaturated ones)            
            theta_now = zeros(n,1);
            % For the nonzero ones, take a binary permutation of -1 and +1:
            theta_now(anti_saturated_index) = flipud(-2*de2bi(k-1,nAntiSaturated)'+1);
            
            % Given this permutation of theta directions, evaluate the
            % associated error:
            tau_dRAnorm = norm(config.model.Mu*A_at_singularity*(theta_now.*dgamma_magnitude_at_singularity));
            
            % If this particular permutation has a lower cost than the last
            % one, take it as the new optimum. That way, we'll end up with
            % the lowest cost and the associated directions
            if(tau_dRAnorm < tau_dRAnorm_minimum)
                tau_dRAnorm_minimum = tau_dRAnorm;
                theta_tilde = theta_now;
            end
        end
        
        %%%%
        %%%% ALGORITHM 2 from paper:
        %%%%        

        % We now have an optimal perturbation direction, but in the next
        % step we will reconsider whether we should apply it, change its
        % sign, or keep the perturbation direction we applied during the previous time step.
        % In some highly redundant gimbal configurations, this prevents
        % chattering.
        
        
        % In principle, we can apply the newly found optimim directions,
        % but we want to prevent a direction switch due to only a minor
        % change in cost (typically only a numerically small/roundoff change).
        % In other words, we apply the newly found directions only if a 
        % signifcant reduction in the cost taudRA can be obtained:        
                  
        % First, find the gimbal indexes that were previously perturbation candidates.
        anti_saturated_index_prev = find(theta_previous ~= 0);

           
        % The 'previous cost', or actually the cost for the current dgamma speeds, but with previous directions is simply:
        tau_dRAnorm_minimum_prev = norm(config.model.Mu*A_at_singularity*(theta_previous.*dgamma_magnitude_at_singularity));        

        % The difference between the minimum cost now, and with the old
        % directions
        
        DeltaTaud = abs(tau_dRAnorm_minimum - tau_dRAnorm_minimum_prev);
        
        
        % If the perturbation candidates are the same as before, we have a
        % similar perturbation as before, but possibly with a different
        % optimal direction for each gimbal.        
        if (isequal(anti_saturated_index,anti_saturated_index_prev)  && DeltaTaud <  config.controller.tauref_min)
            % Here we compare the new and the old cost and apply the new
            % directions if the change is big enough (more than the torque
            % that we consider numerically zero). Otherwise, we keep the
            % old directions to prevent chattering:            
            theta_final = theta_previous;            
        else
            % If we do have to update the directions, look whether we should change the overal sign          
            dgamma_candidate= theta_tilde.*dgamma_magnitude;
            
            if(norm(dgamma_candidate - dgamma_previous) < norm(dgamma_candidate + dgamma_previous))
                theta_final = theta_tilde;
            else
                theta_final = -theta_tilde;                
            end
        end

        % We store the final theta selection for use during the next time
        % sample.
        theta_previous = theta_final;
       
        %%%% Calculating Gimbal Perturbation for the current configuration, 
        %%%% including size and direction, using the formulas in the paper.    
        output.gamma_dot_d = theta_final.*dgamma_magnitude;
        dgamma_previous    = output.gamma_dot_d;       
    end

    %%%% Now that the perturbation rates are known
    %%%% we can compute the control law      
    
    % Pseudoinverse with numerical filtering
    Acirc = A'/(A*A'+delta.alpha*(Um*Um'));
    
    % Controller output
    output.gamma_dot_c = 1/config.model.Mu*Acirc*tau_r+(eye(n)-Acirc*A)*output.gamma_dot_d;           
    
    % It is also useful to study the separate terms in the above sum:
    output.gamma_dot_p        = 1/config.model.Mu*Acirc*tau_r;
    output.gamma_dot_e        = (eye(n)-Acirc*A)*output.gamma_dot_d;    
    
    %%% Obtaining the same result using SVD components, which is useful for
    %%% debugging and other analysis, but can otherwise be omitted.
    gamma_dot_c_svdmethod = zeros(size(output.gamma_dot_c));
    
    % Adding components associated with the torque of the largest singular values
    for j = 1:config.model.outputdimension-1
        gamma_dot_c_svdmethod = gamma_dot_c_svdmethod + 1/config.model.Mu*1/sigmas(j)*V(:,j)*U(:,j)'*tau_r;
    end
    
    % Adding components associated with the torque of the smallest singular value
    gamma_dot_c_svdmethod = gamma_dot_c_svdmethod + 1/config.model.Mu*(sigma_m/(sigma_m^2+delta.alpha))*Vm*Um'*tau_r;
    
    % "Null motion" component associated with the smallest singular value
    gamma_dot_c_svdmethod = gamma_dot_c_svdmethod + (delta.alpha/(sigma_m^2+delta.alpha))*(Vm*Vm')*output.gamma_dot_d;
    
    % Full Null motion components
    for j = (config.model.outputdimension+1):n
        gamma_dot_c_svdmethod = gamma_dot_c_svdmethod + (V(:,j)*V(:,j)')*output.gamma_dot_d;
    end  
    
    % Speed/moment reference debug info
    output.tau_c = config.model.Mu*A*output.gamma_dot_c ;
    output.tau_p = config.model.Mu*A*output.gamma_dot_p ;
    output.tau_d = config.model.Mu*A*output.gamma_dot_d ; 
    output.tau_e = config.model.Mu*A*output.gamma_dot_e ;
    
    
    output.gamma_dot_d_Vspace = V'*output.gamma_dot_d;
    output.gamma_dot_e_Vspace = V'*output.gamma_dot_e;
    
  

    % Output error (neglecting speed tracking error)
    output.tau_error = tau_r-output.tau_c;
    
    % Output components in U Space
    output.tau_d_Uspace = U'*output.tau_d;
    output.tau_e_Uspace = U'*output.tau_e;  
    output.tau_c_Uspace = U'*output.tau_c;
    output.tau_error_Uspace = U'*output.tau_error;  
    
        
    % The same error in SVD components only (can be omitted as well, but useful for insight).
    tau_error_svdmethod = delta.alpha/(sigma_m^2 + delta.alpha)*(Um'*tau_r-config.model.Mu*sigma_m*Vm'*output.gamma_dot_d)*Um;
    
    % Bound the actual gimbal rates (if we do reach these limits, the error
    % expressions given above no longer hold)    
    output.gamma_dot_c_bounded = max(min(config.controller.maxGimbalRate,output.gamma_dot_c),-config.controller.maxGimbalRate);
    
end

