function [wrappedtime, wrappedangles] = wrapangle(time, angles)
% Take a time history of angles and wrap them to [-pi, pi], while removing
% the sudden wrap line at the transition by inserting a NaN sample at the
% transition

    % Sample size
    n = length(time);

    % The actual wrap around
    modangles = mod(angles+pi,2*pi)-pi; 
    
    % Differentiate angles to find the 2pi discontinuity jumps
    % This produces a list of indexes just before the jump.
    tolerance = 0.1;
    jumpindex = find(abs(diff(modangles)) > (2*pi-tolerance))';
    
    % Each segment ends with a jump, except for the last one. So the list
    % of segment end points is
    segmentendindex = [jumpindex; n];
    nSegments = length(segmentendindex);
      
    % This variable keeps the starting point of each segment
    startindex = 1;
    
    % Extract all the segments, and reappend them with NaNs in between.
    for i = 1:nSegments
        
         segmentendindexnow = segmentendindex(i);
         timesegment =      time(startindex:segmentendindexnow);
         anglesegment= modangles(startindex:segmentendindexnow);
         startindex = segmentendindexnow + 1;
         
         if(i == 1)
            % If it's the first segment, just use it.
            wrappedtime   = timesegment;
            wrappedangles = anglesegment;             
         else
            % Prepend a NaN before each next segment
            wrappedtime   = [wrappedtime NaN timesegment];
            wrappedangles = [wrappedangles NaN anglesegment];
         end
    end
    
end

