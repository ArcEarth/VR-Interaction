figure(1);
hold off;
scatter3(X(1:90,1),X(1:90,3),X(1:90,2),'MarkerEdgeColor','none','MarkerFaceColor',[0.75 0.15 0.15])
hold on;
scatter3(X(91:180,1),X(91:180,3),X(91:180,2),'MarkerEdgeColor','none','MarkerFaceColor',[0.15 .75 0.15])
scatter3(X(181:270,1),X(181:270,3),X(181:270,2),'MarkerEdgeColor','none','MarkerFaceColor',[0.15 0.15 .75])

[N,d] = size(X);
q = size(Y,2);

Xu = sum(X,1)/N;
Yu = sum(Y,1)/N;

Xm = X - repmat(Xu,N,1);
Ym = Y - repmat(Yu,N,1);

%0.862413  47672.9 0.262837
alpha = 0.862413;
beta = 47672.9;
gemma = 0.262837;
segmay = 15.0;

K = eye(N) / beta;
diff = (repmat(reshape(Xm,[N,1,d]),1,N,1) - repmat(reshape(Xm,[1,N,d]),N,1,1));
K = K + alpha * exp(reshape(sum(diff.^2,3),[N N]) .* (-gemma * 0.5));

diff = (repmat(reshape(Ym,[N,1,q]),1,N,1) - repmat(reshape(Ym,[1,N,q]),N,1,1));
Ky = exp(reshape(sum(diff.^2,3),[N N]) .* (-segmay * 0.5));
Kdif = zeros(N,N,3);
Kdif(:,:,1) = K;
Kdif(:,:,2) = Ky;

% Xs = Queried X
M = 50;
Mr = (0 : (M-1))' / (M-1);
Mr = repmat(Mr,1,d);
%Xs = repmat(Xm(M,:),M,1) .* Mr + repmat(Xm(1,:),M,1) .* (1 - Mr);
Xs = Xm(1:M,:);

% MxN
diff = (repmat(reshape(Xs,[M,1,d]),1,N,1) - repmat(reshape(Xm,[1,N,d]),M,1,1));
Ks = alpha * exp(reshape(sum(diff.^2,3),[M N]) .* (-gemma * 0.5));

ys = repmat(Yu,M,1) + Ks*(K\Ym);

figure(2);
title('K(X,X)');
imshow(K);

figure(3);
title('K(Xs,X)');
imshow(Ks);

figure(4);
imshow(Kdif);

figure(5);
imshow((ys - Y(1:M,:)) * 100);