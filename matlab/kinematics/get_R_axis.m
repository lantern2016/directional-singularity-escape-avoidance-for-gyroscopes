function R_j_0 = get_R_axis(axis, theta)
% Create rotation matrix for an eigen axis and a principle angle

    % Get axis of unit length
    a = get_unit_vector(axis);

    % Return the rotation matrix
    R_j_0 = eye(3) - tilde(a)*sin(theta)+tilde(a)^2*(1-cos(theta));
    
end

