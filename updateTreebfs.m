function costNew = updateTreebfs(g,start,back_ptrs, cost,ymus)
    n = g.n;
    closed = zeros(n,1);
    open = [start];
    
    while ~isempty(open)
        curr = open(1);
        open(1) = [];
        if closed(curr) ~= 1
            if back_ptrs(curr) ~= 0
                currcoord = g.coord(curr);
                cost(curr) = cost(back_ptrs(curr)) + g.distance(curr, back_ptrs(curr)) + ...
                    infValue(ymus,currcoord(1),currcoord(2));
            end
            closed(curr) = 1;
            for k = 1: length(g.neighbours(curr))
                next_set = g.neighbours(curr);
                next = next_set(k);
                if closed(next) ~= 1
                    open = [open next];
                end
            end
        end
    end
    costNew = cost;
end