function vector6 = tilde6rev(tilde6matrix)
% Reverse tilde6 operator
    tilde3matrix = tilde6matrix(1:3,1:3);
    
    % Determine if last 4 entries are all zeros
    if(isa(tilde6matrix,'numeric'))
        if (norm(tilde6matrix(4,1:4))>1e-6)
            error('Input must be a Twist Tilde Matrix')
        end
    else
        if(~isequaln(tilde6matrix(4,1:4),sym([0,0,0,0])))
            error('Input must be a Twist Tilde Matrix')
        end
    end
    
    % If they are zero (by the tolerance), perform the reverse tilde
    % operator
    vector3 = tilde3rev(tilde3matrix);
    vector6 = [vector3; tilde6matrix(1:3,4)];
end