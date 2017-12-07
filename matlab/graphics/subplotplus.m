function handle = subplotplus(totalRows, totalColumns, currentRows, currentColumns)
% Same functionality as subplot, but with simpler index values so you don't
% have to count/change the plot numbers every time you add something new.
% Also works when selecting multiple columns or rows for a plot. Example
% usage: 
%
% % Make a subplot for a 4*3 grid, with the current subplot filling the
% % plot at row 1, column 2:
% subplotplus(4, 3, 1, 2)
%
% % Make a subplot for a 4*3 grid, with the current subplot filling the
% % top half of the first column: 
% subplotplus(4, 3, 1:2, 1)
%
% % Make a subplot for a 4*3 grid, with the current subplot filling the
% % entire second row. This can be done in two equivalent ways: 
% subplotplus(4, 3, 2, 1:3)
% subplotplus(4, 3, 2, 'all')

   % The basic plotnumbers grid is:
   grid = reshape(1:(totalRows*totalColumns),[totalColumns totalRows])';
    
   % Specify currentRows, currentColumns when the user has specified 'all'.
   if(strcmp(currentRows, 'all'))
       currentRows = 1:totalRows;
   end
   if(strcmp(currentColumns, 'all'))
       currentColumns = 1:totalColumns;
   end
   
   % The plot numbers are all the numbers corresponding to the intersection
   % of the currentRows and current Columns
   
   plotNumbers = []; % Empty array of plotnumbers  
   
   % Get those intersections and extract the plot numbers
   for r_index = 1:length(currentRows)
       for c_index = 1:length(currentColumns)
            currentPlotNumber = grid(currentRows(r_index), currentColumns(c_index));
            plotNumbers = [plotNumbers currentPlotNumber]; %#ok<AGROW> Really, this is fine.
       end
   end
   
   % Return the handle. Now it's just a normal subplot.
   handle = subplot(totalRows, totalColumns, plotNumbers);
end

