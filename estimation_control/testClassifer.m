bldata_test = load('march17_david/putty_badLeftDavid.log');
brdata_test= load('march17_david/putty_badRightDavid.log');
gdata_test = load('march17_david/putty_goodDavid.log');

%cols = [2 3 4 10 11 12];
%X = gdata_mar13(:, cols).*9.81;
bl_test = -1*ones(length(bldata_test), 1);
br_test = 1*ones(length(brdata_test), 1);
g_test = zeros(length(gdata_test), 1);

cols = [2 3 4 8 9 10];
X_test = [gdata_test(:, cols); bldata_test(:,cols); brdata_test(:,cols)];
Y = [g_test; bl_test; br_test];

X_ = zeros(size(X_test));
avg = X_test(1,:);
X_(1,:) = X_test(1,:);

for i=2:length(X_test)
    avg = (3.*avg + X_test(i,:))./4;
    X_(i,:) = avg;
end

%Y = -1*ones(length(X), 1);
%br = 1*ones(length(brdata), 1);
n = length(X_);
j = 1;
GoodTest = 0;
goodtree = 0;
for i=1:100%goodTrees
    Ynew = predict(ctree{i}, X_);
    GoodTest(i) = sum(Ynew == Y);
    if (GoodTest(i)/n > 0.978)
        goodtree(j) = i;
        j = j+1;
    end
end

%GOODTREES = goodtree;
figure(5);
bar((GoodTest)/n); 
ylim([0.5 1.000]);
xlim([0.5 100.5]);
title('All Data Correctly Classified');
xlabel('Iterations');
ylabel('Percentage');

picked_tree=intersect(goodtree, goodTrees)
for i=picked_tree
    ctree{i}.NumNodes
end
