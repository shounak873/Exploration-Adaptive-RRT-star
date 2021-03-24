function [pathcost, infcost,pathvec] = plotTree(G, goal,solSet,cost, back_ptrs, iter,ymus)
    pathcost = 0; %Length of path
    infcost = 0;  %Information gathered along path
    hold on;
    set(gca,'TickLabelInterpreter','latex');

%     G.add_node([goal(1),goal(2)]);
%     G.plot('labels','NodeLabelSize',8,'NodeSize',4,'EdgeColor','green')
    edgemat = G.edgelist;
    for k = 1: size(edgemat,2)
       if ~isnan(edgemat(1,k)) 
         p1 = G.coord(edgemat(1,k));
         p2 = G.coord(edgemat(2,k));
         plot([p1(1), p2(1)], [p1(2), p2(2)],'g-')
         plot(p1(1), p1(2),'ko','MarkerSize', 2)
         plot(p2(1), p2(2),'ko','MarkerSize', 2)
       end
    end
    grid off;
    box on;
    [minCost, ind] = min(cost(solSet));
    solSetSorted = solSet(ind);
%     back_ptrs(G.n) = solSetSorted(1);
    gdist = norm(G.coord(solSetSorted) - [goal(1);goal(2)]);
%     cost(G.n) = cost(solSetSorted(1)) + gdist;
    curr = solSetSorted;
    %     disp(" Path finding time ")
    %     tic;

    % calculate the path using back pointers , also add the intermediete points
    % using back_configs

    pathvec = [curr];

    while curr ~= 1
        pathvec = [back_ptrs(curr), pathvec];
        curr = back_ptrs(curr);
    end
       
    
    for I = 1: length(pathvec)
        ps = G.coord(pathvec(I));
        infcost = infcost + (max(max(ymus)) - infValue(ymus,ps(1),ps(2)));
        if I>1
            pathcost = pathcost + G.distance(I,I-1);
        end 
    end

    G.highlight_path(pathvec,'NodeFaceColor','y')
    radius = 0.05;

    h = circle(goal(1),goal(2),radius);
    h.Color = 'r';
    
    title(['Iteration (Samples) - ', num2str(iter)],'Interpreter','latex') 
%     text(m(1)-0.15,n(1) + 0.15,'\textbf{start}','Interpreter','latex')
%     text(m(2)-0.15,n(2) + 0.15,'\textbf{goal}','Interpreter','latex')
end