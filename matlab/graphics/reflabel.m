function  reflabel(labelString)
% Add label to lower left corner

    ax = gca; 
    
    
    xpos = 0.04;
    ypos = 0.04;
    
    formattedstring = ['$\textbf{' labelString ')}$'];
    
    text(xpos,ypos,formattedstring,'units','centimeters','VerticalAlignment','bottom','Interpreter','latex');

end

