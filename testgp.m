hold on;
x = [];
y = [];
xlim([0,1]);
ylim([0,1]);
for i = 1:20
    [c, d] = ginput(1);
    x = [x;c];
    y = [y;d];
    plot(c,d,'ko');
end
hold off;
X = [x y];
rng(0,'twister');
Y = randi([3 5],20,1);

% produce the test set for regression

% set the hyperparameters
covfunc = {@covMaterniso, 3};
ell = 1/8; sf = 1;
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

pcolor(Xtest1,Xtest2, reshape(ymu,size(Xtest1)))
shading('flat')