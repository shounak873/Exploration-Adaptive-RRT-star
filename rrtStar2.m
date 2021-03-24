close all;
clear;
%read the obstacles from this text file
% B = readmatrix('obs_prm.txt');
% r = readmatrix('r_prm.txt');
pathvec = zeros(5,1);
infvec = zeros(5,1);
solSet = [];
rdist = 0;
eta = 0.05;
iter = 0;

figure();
set(gca,'TickLabelInterpreter','latex');
box on;
n = 0;
hold on;
box on;
xlim([0,1]);
ylim([0,1]);
% select the start and goal locations using ginput()
% enter also th orientaions with keyboard
disp('Please point the start and goal locations')
m = [];
n = [];
x = [];
y = [];
for i = 1:2
    [p, q] = ginput(1);
    plot(p,q, 'go','MarkerFaceColor','g');
    if i ==1
        text(p+0.05,q+0.05,'start','Interpreter','latex');
    else
        text(p+0.05,q+0.05,'goal','Interpreter','latex');
    end
    m = [m p];
    n = [n q];
end

center = [(m(1)+m(2))/2, (n(1)+n(2))/2];
c = norm([m(1)-m(2), n(1)-n(2)]);
ang = atan2(n(1)-n(2),m(1)-m(2));
rot = [cos(ang) -sin(ang); sin(ang) cos(ang)];
a = (c + 0.2)/2;
b = sqrt((c + 0.2)^2 - c^2)/2;
t = linspace(0,2*pi) ;
xe = center(1) + a*cos(t)*cos(ang) - b*sin(t)*sin(ang);
ye = center(2) + b*sin(t)*cos(ang) + a*cos(t)*sin(ang);
% plot(center(1), center(2),'ro');
% plot(xe,ye,'r--')
foc1 = [m(1);n(1)];
foc2 = [m(2),n(2)];


rng(0,'twister');
Y = randi([1 5],5,1);
disp('Please point the prior information locations')
for i = 1:5
    [c, d] = ginput(1);
    x = [x;c];
    y = [y;d];
    plot(c,d,'rs','MarkerFaceColor','r');
    text(c+0.05,d+0.05,num2str(Y(i)),'Interpreter','latex');
end
hold off;
tic;

% GP Regression 
X = [x y];


% produce the test set for regression

% set the hyperparameters
covfunc = {@covMaterniso, 2};
ell = 1/20; sf = 1;
hyp.cov = log([ell; sf]);
likfunc = @likGauss;
sn = 0.1;
hyp.lik = log(sn);

% implement regression
% testdata
[Xtest1, Xtest2] = meshgrid(0:0.01:1, 0:0.01:1);
 Xtest = [Xtest1(:) Xtest2(:)];

% implement regression 
[ymu ys2 fmu fs2] = gp(hyp, @infExact, [], covfunc, likfunc, X, Y, Xtest);

ymus = reshape(ymu,size(Xtest1));

%start with the init node --add to graph
initPose = [m(1), n(1)]; 
G = PGraph();
G.add_node(initPose);

infinit = infValue(ymus,initPose(1),initPose(2));

infCost(1) = infinit;
cost(1) = infinit;         % vector for storing cost from start of each node
back_ptrs = [];        % vector for storing back_ptrs of each node

figure();
pcolor(Xtest1,Xtest2, reshape(ymu,size(Xtest1)))
shading('flat')
title('Information map','Interpreter','latex')
c = gray;
c = flipud(c);
colormap(c);
set(gca,'TickLabelInterpreter','latex');
box on;
h = colorbar;
set(h,'TickLabelInterpreter','latex');


%ellipsoid parameters

%%
for i = 1:5000         
    
    [xrand, yrand] = sampleFree();  % sample position 
    
    % define the new node
    
    [d, w] = G.distances([xrand, yrand]);
    
    nearest = w(1);
    
    neareststate = G.coord(nearest);
    xnearest = neareststate(1);
    ynearest = neareststate(2);
    
    [xnew, ynew] = steer(neareststate,[xrand,yrand]');
    new = [xnew, ynew]';
    infNew = infValue(ymus,xnew,ynew); 
    
    % if free then add this to tree

    iter = iter+1;
    [d, w] = G.distances([xnew, ynew]);
    rdist = min(sqrt(log(G.n)/G.n),eta);
    
    W = w(d < rdist);
    G.add_node(new);
    curr = G.n;
    dist = norm(new - neareststate);        
    minCost = cost(nearest) + dist; %+ infValue(ymus,xnew,ynew);
    minNode = nearest; 

    %FIND BEST NODE FOR ADDING NEW EDGE
    for j = 1: length(W)
        if (cost(W(j)) + norm(new - G.coord(W(j)))) < minCost
            minCost = cost(W(j)) + norm(new - G.coord(W(j))) ;
            minNode = W(j);
        end
    end
    
    cost(G.n) = minCost;
    G.add_edge(minNode,G.n);
    back_ptrs(G.n) = minNode;


    %REWIRING THE TREE
    for j = 1: length(W)
        coordn = G.coord(W(j));
        if (cost(G.n) + norm(new - G.coord(W(j)))) < cost(W(j))
            %back_ptrs(W(j)) = G.n;
            edges = G.edgelist;
            for k = 1: size(edges,2)
                if (edges(1,k) == W(j)||edges(1,k) == back_ptrs(W(j))) && ...
                      (edges(2,k) == W(j)||edges(2,k) == back_ptrs(W(j)))
                    edgeInd = k;
                  break;
                end
            end
            cost(W(j)) = cost(G.n) + norm(new - G.coord(W(j)));
            G.delete_edge(edgeInd);
            G.add_edge(W(j),G.n);
            back_ptrs(W(j)) = G.n;
            start = W(j);
            % Update the cost of sub-tree starting from W(j) 
            cost = updateTreebfswoinfo(G,start,back_ptrs, cost,ymus);                  
        end       
    end

    gdist = norm(new - [m(2),n(2)]');
    % if close to goal then add new to solSet
    if gdist < 0.05
%             disp("Goal found!");
        solSet = [solSet,G.n];
        cost(solSet);
    end 

    if iter == 1000
        disp('1000!')
        figure();
        hold on;
        pcolor(Xtest1,Xtest2, reshape(ymu,size(Xtest1)))
        shading('flat')
        c = gray;
        c = flipud(c);
        colormap(c);
        [pathcost, infcost] = plotTree(G, [m(2), n(2)], solSet,cost, back_ptrs,iter,ymus);
        pathvec(1) = pathcost;
        infvec(1) = infcost;
        
    elseif iter == 2000
        disp('2000!')
        figure();
        hold on;
        pcolor(Xtest1,Xtest2, reshape(ymu,size(Xtest1)))
        shading('flat')
        c = gray;
        c = flipud(c);
        colormap(c);
        [pathcost, infcost] = plotTree(G, [m(2), n(2)], solSet,cost, back_ptrs,iter,ymus);
        pathvec(2) = pathcost;
        infvec(2) = infcost;
        
    elseif iter == 3000    
        disp('3000!')
        figure();
        hold on;
        pcolor(Xtest1,Xtest2, reshape(ymu,size(Xtest1)))
        shading('flat')
        c = gray;
        c = flipud(c);
        colormap(c);       
        [pathcost, infcost] = plotTree(G, [m(2), n(2)], solSet,cost, back_ptrs,iter,ymus);
        pathvec(3) = pathcost;
        infvec(3) = infcost;
        
    elseif iter == 4000    
        disp('4000!')
        figure();
        hold on;
        pcolor(Xtest1,Xtest2, reshape(ymu,size(Xtest1)))
        shading('flat')
        c = gray;
        c = flipud(c);
        colormap(c);     
        [pathcost, infcost] = plotTree(G, [m(2), n(2)], solSet,cost, back_ptrs,iter,ymus);
        pathvec(4) = pathcost;
        infvec(4) = infcost;
        
    elseif iter == 5000    
        disp('5000!')
        figure();
        hold on;
        pcolor(Xtest1,Xtest2, reshape(ymu,size(Xtest1)))
        shading('flat')
        c = gray;
        c = flipud(c);
        colormap(c);     
        [pathcost, infcost] = plotTree(G, [m(2), n(2)], solSet,cost, back_ptrs,iter,ymus);
        pathvec(5) = pathcost;
        infvec(5) = infcost;
        toc;
        break;
    end

end

