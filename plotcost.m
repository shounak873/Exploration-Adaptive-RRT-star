

hold on;
title('Cost evolution with incresing iterations','Interpreter','latex');
iter = [1000, 2000, 3000, 4000, 5000];
% limcost = 1.4*ones(5,1);
subplot(2,1,1)
% hold on;
box on;
plot(iter, pathvec,'rs--');
% plot(iter,limcost,'k--');
% text(3500,1.45,'budget','Interpreter','latex');
ylabel('Length of path','Interpreter','latex');
xlabel('Iterations','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');
subplot(2,1,2)
plot(iter, infvec,'bs--');
box on;
ylabel('Cumulative Information score','Interpreter','latex');
xlabel('Iterations','Interpreter','latex');
ylim([0,1]);
set(gca,'TickLabelInterpreter','latex');