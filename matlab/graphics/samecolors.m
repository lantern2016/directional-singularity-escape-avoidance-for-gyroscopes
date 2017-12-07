function samecolors(handles1, handles2)
% Gives the lines of handles2 the same color as those in handles1. This is
% useful when plotting signals and corresponding references which are
% dotted, but should have the same colors as the original signals.
    
    % Determine that the number of lines are equal
    nLines1 = length(handles1);
    nLines2 = length(handles2);

    if(nLines1 ~= nLines2)
        error('Both plots must have the same number of lines')
    end

    % Make the colors equal
    for c = 1:nLines1
        handles2(c).Color = handles1(c).Color;
    end
end

