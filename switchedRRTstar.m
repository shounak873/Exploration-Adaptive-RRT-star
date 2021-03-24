load infrrtstar.mat;
load normalrrtstar.mat;
iter = 5000;
[Xtest1, Xtest2] = meshgrid(0:0.01:1, 0:0.01:1);

solSet = [];
for i = 1:G.n
    node = G.coord(i);
    dist = norm(node - [m(2),n(2)]');
    if dist < 0.05
        solSet  = [solSet, i];
    end
end

figure();
hold on;
pcolor(Xtest1,Xtest2,iymus)
shading('flat')
c = gray;
c = flipud(c);
colormap(c);     
[pathcost, infcost, pathvec] = plotTree(G, [m(2), n(2)], solSet,cost, back_ptrs,iter,ymus);

pathvecc = [];
for j = 1:length(pathvec)
    pathvecc = [G.coord(pathvec(j)), pathvecc];
end


figure();
hold on;
pcolor(Xtest1,Xtest2, iymus)
shading('flat')
c = gray;
c = flipud(c);
colormap(c);     
[ipathcost, iinfcost, ipathvec] = plotTree(infG, [m(2), n(2)], isolSet,icost, infback_ptrs,iter,iymus);

ipathvecc = [];
for j = 1:length(ipathvec)
    ipathvecc = [G.coord(ipathvec(j)), ipathvecc];
end

m = length(pathvec);
ind = round(m/2);
changepoint = pathvec(ind);

coordpt = G.coord(changepoint);

[d,w] = infG.distances(coordpt);
newpt = w(1);
curr = newpt;
newpath = [infG.coord(newpt)];

while curr ~= 1
    newpath = [infG.coord(infback_ptrs(curr)), newpath];
    curr = back_ptrs(curr);
end

for j = ind:length(pathvec)
    newpath = [newpath,G.coord(pathvec(j))];
end

plot(newpath(1,:),newpath(2,:),'ms-','MarkerFaceColor','m','MarkerSize',7,'MarkerEdgeColor','k');


figure();
hold on;
pcolor(Xtest1,Xtest2,iymus)
shading('flat')
c = gray;
c = flipud(c);
colormap(c);
l1 = plot(pathvecc(1,:),pathvecc(2,:),'ko-','MarkerFaceColor','g','MarkerSize',7,'MarkerEdgeColor','k');
l2 = plot(ipathvecc(1,:),ipathvecc(2,:),'ko-','MarkerFaceColor','y','MarkerSize',7,'MarkerEdgeColor','k');
l3 = plot(newpath(1,:),newpath(2,:),'cs-','MarkerFaceColor','c','MarkerSize',7,'MarkerEdgeColor','k');
plot(coordpt(1),coordpt(2),'gs','MarkerSize',12)
set(gca,'TickLabelInterpreter','latex');
lg = legend([l1,l2,l3],'Shortest path','Explorative path','Switched mode path','Interpreter','Latex');
lg.FontSize = 10;
box on;


