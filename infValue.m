function value = infValue(ymus,x,y)
    %penalty for less information
    xRound = round([x, y],2);
    row = round(xRound(2)*100)+1;
    col = round(xRound(1)*100)+1; 
    value = max(max(ymus)) - ymus(row,col);
end
