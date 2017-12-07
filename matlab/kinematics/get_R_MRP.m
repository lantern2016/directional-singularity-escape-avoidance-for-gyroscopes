function R_j_0 = get_R_MRP(sigma)
% Create rotation matrix for an MRP vector

    % Return the rotation matrix that transforms vectors in the inertial frame 0 to body frane j.
    R_j_0 = eye(3)+4/(1+sigma'*sigma)^2*((sigma'*sigma-1)*tilde(sigma)+2*tilde(sigma)^2);
end

