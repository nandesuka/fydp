clear all; clc;
% bldata = load('march9/putty_badleft.log');
% brdata = load('march9/putty_badright.log');
% gdata = load('march9/putty_good.log');
% %g1data = load('march13/putty_good.log');
% %g2data = load('march13/putty_good2.log');
% 
% bl = -1*ones(length(bldata), 1);
% br = 1*ones(length(brdata), 1);
% g = zeros(length(gdata), 1);
% %+length(g1data)+length(g2data)
% 
% data = [bldata; brdata; gdata];
% Y = [bl; br; g];
% sens1 = data(:,2);
% sens2 = data(:,11);
% 
% X = [sens1, sens2];

bldata_mar16 = load('march16/putty_badLeft.log');
brdata_mar16 = load('march16/putty_badRight.log');
gdata_mar16 = load('march16/putty_goodSquats.log');

bl_mar16 = -1*ones(length(bldata_mar16), 1);
br_mar16 = 1*ones(length(brdata_mar16), 1);
g_mar16 = zeros(length(gdata_mar16), 1);

bldata_test = load('march17/putty_badLeft.log');
brdata_test= load('march17/putty_badRight.log');
gdata_test = load('march17/putty_goodS.log');
% 
% %cols = [2 3 4 10 11 12];
% %X = gdata_mar13(:, cols).*9.81;
bl_test = -1*ones(length(bldata_test), 1);
br_test = 1*ones(length(brdata_test), 1);
g_test = zeros(length(gdata_test), 1);

bldata_test2 = load('march17_/putty_badLeft.log');
brdata_test2 = load('march17_/putty_badRight.log');
gdata_test2 = load('march17_/putty_good.log');
% 
% %cols = [2 3 4 10 11 12];
% %X = gdata_mar13(:, cols).*9.81;
bl_test2 = -1*ones(length(bldata_test2), 1);
br_test2 = 1*ones(length(brdata_test2), 1);
g_test2 = zeros(length(gdata_test2), 1);

cols = [2 3 4 8 9 10];
%X_raw = [gdata_mar16(:, cols); bldata_mar16(:,cols); brdata_mar16(:,cols)];
%Y = [g_mar16; bl_mar16; br_mar16];

X_raw = [gdata_mar16(:, cols); bldata_mar16(:,cols); brdata_mar16(:,cols); gdata_test(:, cols); bldata_test(:,cols); brdata_test(:,cols); gdata_test2(:, cols); bldata_test2(:,cols); brdata_test2(:,cols)];
Y = [g_mar16; bl_mar16; br_mar16; g_test; bl_test; br_test; g_test2; bl_test2; br_test2];

X = zeros(size(X_raw));
avg = X_raw(1,:);
X(1,:) = X_raw(1,:);
for i=2:length(X)
    avg = (avg + X_raw(i,:))./2;
    X(i,:) = avg;
end

n = length(X);

%svmStruct = svmtrain(X,Y,'ShowPlot',true);
%Md1 = fitcnb(X, Y, 'ClassNames', [-1 1 0]);
%sum(Y == predict(Md1, X))
% 
trainPer = 0.5;
testPer = 0.5;

%% Optimal Tree Depth %%
% rng('default');
% leafs = linspace(1,50, 50);
% N = numel(leafs);
% err = zeros(N,1);
% for n=1:N
%     t = fitctree(X,Y,'CrossVal','On',...
%         'MinLeafSize',leafs(n));
%     err(n) = kfoldLoss(t);
% end
% plot(leafs,err);
% xlabel('Min Leaf Size');
% ylabel('cross-validated error');



GoodTrees = 0;
j = 1;
for i=1:100
    [trainInd, valInd, testInd] = dividerand(n, trainPer, 0, testPer);
  
    ctree{i} = fitctree(X(trainInd,:), Y(trainInd,:), 'MinLeafSize', 10);
  
    Ynew = predict(ctree{i}, X(testInd,:));
    GoodTest(i) = sum(Ynew ==Y(testInd,:));
    Ynew = predict(ctree{i}, X(trainInd,:));
    GoodTrain(i) = sum(Ynew == Y(trainInd,:));
    
    AllData(i) = GoodTest(i)+GoodTrain(i);
    if (AllData(i)/n > 0.99)
        GoodTrees(j) = i;
        j = j + 1;
    end
    NumNodes(i) = ctree{i}.NumNodes;
end

GoodTrees

figure(1);
bar(NumNodes);
xlim([0.5 100.5]);
title('Number of Nodes in Tree');
xlabel('Iterations');
ylabel('Num Nodes');

figure(2);
bar((GoodTest/(n*testPer))); 
ylim([0.98 1.000]);
xlim([0.5 100.5]);
title('Test Data Correctly Classified');
xlabel('Iterations');
ylabel('Percentage');

figure(3);
bar((GoodTrain/(n*trainPer))); 
ylim([0.98 1.000]);
xlim([0.5 100.5]);
title('Trainning Data Correctly Classified');
xlabel('Iterations');
ylabel('Percentage');

figure(4);
bar((GoodTrain+GoodTest)/n); 
ylim([0.98 1.000]);
xlim([0.5 100.5]);
title('All Data Correctly Classified');
xlabel('Iterations');
ylabel('Percentage');

% 
% ctree = fitctree(X, Y);
% Ynew = predict(ctree, X);
% GoodTest = sum(Ynew == Y);
% 
% view(ctree, 'mode', 'graph');
% 
% %[trainInd, valInd, testInd] = dividerand(n, 0.5, 0, 0.5);
% %ctree = fitctree(X(trainInd,:), Y(trainInd,:));
%  
% figure;
% plot(X(:,1:3));
% figure;
% plot(X(:,4:6));
% 
% 
view(ctree{1}, 'mode', 'graph');