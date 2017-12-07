function output_vector = get_unit_vector(input_vector)
% Turn a vector into unit vector.

    
    if(isa(input_vector,'numeric'))
        %Allowable tolerance below which the vector is considered to be zero:
        zero_tolerance = 1e-6;

        % Check that we are not dealing with a zero vector
        if(norm(input_vector) < zero_tolerance)
            error('Axis must be of finite length')
        end
    else
        % For symbolic variables, the vector cannot be zero
        if(isequaln(input_vector,sym(zeros(size(input_vector)))))
            error('Axis must be of finite length')
        end
    end    

    % Calculate and return the unit vector
    output_vector = input_vector/norm(input_vector);    
end

