function vector3 = tilde3rev(tilde3matrix)
% Reverse tilde3 operator

    %For a skew symmetric matrix, this matrix should be zero:
    error_matrix = simplify(expand(tilde3matrix)+expand(tilde3matrix)'); 
    
    if(isa(error_matrix,'numeric'))
        %Allowable tolerance:
        skew_symmetry_tolerance = 1e-6;

        %Check if matrix is skew symmetric by this norm
        if(norm(error_matrix,inf) > skew_symmetry_tolerance)
            error('Input must be skew symmetric')
        end
    else
        if(~isequaln(error_matrix,sym(zeros(3,3))))
            error('Input must be a Twist Tilde Matrix')
        end
    end    

    
    %If skew symmetric, return the associated 3x1 vector
    vector3 = [ tilde3matrix(3,2);
                tilde3matrix(1,3);
                tilde3matrix(2,1)];  

end