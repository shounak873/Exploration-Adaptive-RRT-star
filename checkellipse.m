figure();
hold on;
xlim([0,1]);
ylim([0,1]);
m = [];
n = [];
for i = 1:2
    [p, q] = ginput(1);
    plot(p,q, 'ko','MarkerSize',5);
    m = [m p];
    n = [n q];
end
major = 0.8;
plot([m(1),m(2)], [n(1),n(2)],'k','LineWidth',3);

for i = 1:5000
    point = rand(2,1);
    if norm(point - [m(1);n(1)]) + norm(point - [m(2);n(2)]) <= major
        plot(point(1), point(2),'ro')
    else
        plot(point(1), point(2),'go')
    end
end
hold off;