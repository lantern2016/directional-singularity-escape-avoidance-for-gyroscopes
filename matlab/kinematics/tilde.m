function tilde = tilde(in)
% Tilde operator (wrapper function)
% Returns 3x3 tilde matrix for a 3x1 input.
% Returns 4x4 tilde matrix for a 6x1 input.
    if(length(in) == 6)
        tilde = tilde6(in);
    elseif(length(in) == 3)
        tilde = tilde3(in);
    else
        error('Incorrect input');
    end
end