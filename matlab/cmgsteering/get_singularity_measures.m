function [delta, A, G, H, U, S, V] = get_singularity_measures(t, y, tau_r, config)
% Obtain the singularity measures for the current tau_r and A matrix.

    % Empty delta structure for initialization purposes
    delta.man         = 0;
    delta.O           = NaN;
    delta.SDA         = 0;
    delta.pDSR        = NaN; 
    delta.zat         = NaN;
    delta.uns         = NaN;
    delta.Wie         = 0;
    delta.alpha       = 0;
    delta.alpha_filter= 0;
    delta.lambda      = 0;
    delta.p_i         = NaN(config.model.nCMGs,1);
    delta.d_i         = NaN(config.model.nCMGs,1);
    delta.p_i_bar     = NaN;
    delta.RA          = NaN;
    delta.z           = zeros(config.model.outputdimension,1);
    delta.psi         = NaN(config.model.nCMGs,1);
        
    % For initialization, only return this 'empty' structure
    if(isempty(t))    
        return;
    end
    
    % Gimbal frame vectors in the 3D case
    [A, G, H] =get_AGH(y, config); 
    
    % For 3D applications, the A, G and H matrices remain unchanges.
    % For 2D applications, we take only the first two rows of these
    % matrices.
    m = config.model.outputdimension;
    A = A(1:m,:);
    G = G(1:m,:);
    H = H(1:m,:);
       
    % Also reduce the reference moment to the appropriate dimension
    tau_r = tau_r(1:m);
    
    % Number of CMGs
    n = config.model.nCMGs;
    
    % Singular value decomposition
    [U,S,V] = svd(A);    
    
    % Change the sign of the m'th direction if necessary (Without loss of
    % generality. Insightful for plots, in order to prevent sign changes)
    if( U(:,   m)'*tau_r < 0)
        U(:,   m) = -U(:,m);
        V(:,   m) = -V(:,m);
    end
    
    % Change the sign of the m+1'th direction if necessary (Without loss of
    % generality. Insightful for plots, in order to prevent sign changes)
    persistent Vprev
    if(isempty(Vprev))
        Vprev = V;
    end
    for j = m+1:n    
        vnow  = V(:,j);
        vprev = Vprev(:,j);        
        if( norm(vnow+vprev) < norm(vnow-vprev))
            V(:,   j) = -V(:,   j);
        end        
    end    
    Vprev = V;    
    
    % Find properties of smallest singular value
    sigmas     = diag(S);
    s_smallest = sigmas(end);
    U_smallest = U(:,   end);    
       
    % Singularity measures based on A
    delta.man = (length(sigmas)/n)^(length(sigmas)/2)*sqrt(abs(det(A*A')));% Dimensionless manipulability measure (Valk et al., 2018)
    delta.SDA = (length(sigmas)/n)^(1/2)             *s_smallest;          % Dimensionless smallest singular value (Ford, Hall, 2000)
    
    % Damping (Valk et al., 2018)
    delta.alpha = config.controller.sigma_min^2*exp(config.controller.beta*(config.controller.sigma_min^2-s_smallest^2));
    delta.alpha_filter = s_smallest^2/(delta.alpha + s_smallest^2);
    
    %%% Wie singularity measure/indicator (Wie, 2001/2005)
    delta.lambda = config.controller.k_lambda_1*exp(-config.controller.k_lambda_2*det(A*A'));

    % Some singularity measures are functions of the direction of the
    % torque reference. These are only defined for (numerically) nonzero torque:
    if(norm(tau_r) > config.controller.tauref_min)
        % For a nonzero torque, the direction is known and the singularity
        % measures are well defined:
        
        % Unit reference moment
        tau_ref_hat = tau_r/norm(tau_r);
        
        % Previously existing singularity measures
        delta.O   = ((tau_r'*A)*(A'*tau_r))/(tau_r'*tau_r)/n;  % Orthogonality index (Oh, Vadali)
        delta.pDSR= prod(tau_r'*H >= zeros(1,n));              % Saturation switch   (Berry 2016 DSR)
        delta.zat = 1+(  delta.pDSR)*(delta.O-1);              % Saturated   RA      (Berry 2016 DSR)
        delta.uns = 1+(1-delta.pDSR)*(delta.O-1);              % Unsaturated RA      (Berry 2016 DSR)
        
        % Compute the potential for each gimbal (Valk et al., 2018)
        for i = 1:n  
            bi_abs= sqrt(1-(tau_ref_hat'*G(:,i))^2);
            delta.p_i(i) = 1/2*(bi_abs - H(:,i)'*tau_ref_hat);
            delta.d_i(i) = (H(:,i)'*tau_ref_hat < 0)*(H(:,i)'*tau_ref_hat)^2;
            
            % Psi values
            delta.psi(i) = abs(acos(1/bi_abs*tau_ref_hat'*H(:,i)));

        end    
 
        % Total gimbal potential (Valk et al., 2018)
        delta.p_i_bar = mean(delta.p_i);          
        
        % Reference aligned singularity measure as a smooth measure for error
        
        % Kappa values
        kappa = config.controller.sigma_acc^2*exp(config.controller.beta*(config.controller.sigma_acc^2-sigmas.^2));
        
        % Filter values
        delta.z = sigmas.^2./(sigmas.^2+kappa);
        
        % Tracking index (Valk et al., 2018)
        delta.RA = 1 - sqrt(tau_ref_hat'*U*(eye(m)-diag(delta.z))^2*U'*tau_ref_hat);
         
    end   
end

