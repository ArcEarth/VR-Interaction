Jk = 15;
J = size(X,2) / 3;

NatrualCon = [ 1 2; 1 3; 1 4; 1 7; 4 5; 4 7; 5 6; 7 8 ; 8 9; 1 10; 10 11; 10 13 ; 11 12; 1 13; 13 14 ; 14 15 ; 10 13];
SoftCon = [ 2 3; 2 4; 2 7; 3 4; 3 7 ; 2 10 ; 2 13; 3 10; 3 13; 4 10; 4 13; 7 10; 7 13];
MotionCon = [1 11; 1 14; 4 8 ; 5 7];

NatrualCon(:,1) = (NatrualCon(:,1) - 1) * Jk;
NatrualCon = sum(NatrualCon,2);
SoftCon(:,1) = (SoftCon(:,1) - 1) * Jk;
SoftCon = sum(SoftCon,2);
MotionCon(:,1) = (MotionCon(:,1) - 1) * Jk;
MotionCon = sum(MotionCon,2);

Index = zeros(Jk);
Index(tril(~eye(Jk))) = 1:J;
Index = Index + Index';

Ideal = eye(Jk) * 0.5;
Ideal(NatrualCon) = 1;
Ideal(SoftCon) = 0.8;
Ideal(MotionCon) = 0.7;
Ideal = Ideal + Ideal';
figure(7);
image(1-Ideal,'CDataMapping','scaled');
colorbar;
hold on;
for i = 1: Jk
    for j = 1 : Jk
        if (Index(i,j) ~= 0)
            text(i - 0.3,j,sprintf('%d',Index(i,j)),'Color','w');
        end
    end
end
hold off;
figure(11);
IEj = tril(1 - eye(Jk));
IEj(tril(~~IEj)) = Ej;
IEj = IEj + IEj';
image(IEj,'CDataMapping','scaled');
colorbar;
hold on;
for i = 1: Jk
    for j = 1 : Jk
        if (Index(i,j) ~= 0)
            text(i - 0.3,j,sprintf('%d',Index(i,j)),'Color','w');
        end
    end
end
hold off;

N= size(Xs,1);
Xr = reshape(Xs,[N,3,J]);
Xr = sum(Xr.*Xr,2);
Xr = reshape(Xr, [N,J]);
Xr = sqrt(Xr);

MeanXr = median(Xr);
Xr = Xr - repmat(MeanXr,size(Xr,1),1,1);
% 
% Xr = sum(Xr.*Xr,2);
% Xr = reshape(Xr, [N,J]);
% Xr = sqrt(Xr);
VXr = sum(Xr > 0.014) ./ size(Xr,1);
eSj = VXr < 0.1; % Relative Static 
Xr = reshape(Xs,[N,3,J]);
MeanXr = reshape(median(Xr),[3,J]);
meanYr = reshape(median(Y),[Yd,C]);
meanYr = meanYr(1:3,:);

Sr = zeros(J,C);

%Normalize Mean-X in R2
MeanXr = MeanXr(:,eSj);
temp = diag(1 ./ sqrt(dot(MeanXr,MeanXr)));
MeanXr = MeanXr * temp;
%Normalize Mean-Y in R2
temp = diag(1 ./ sqrt(dot(meanYr,meanYr)));
meanYr = meanYr * temp;

Sr(eSj,:) = MeanXr' * meanYr;
figure(10);
image(Sr,'CDataMapping','scaled');
colorbar;
title('Sptial Correlation');
%Xc = diag(1 ./ max(VXr));

b = tril(~eye(Jk));
Tri = zeros(Jk);
Tri(b) = VXr;
Tri = Tri + Tri';
% Tri = repmat(Tri,1,1,3);
% Tri(:,:,2) = 1 - Tri(:,:,2);
% Tri(:,:,3) = zeros(Jk);

Tri(Tri < 0.105) = -0.5;
figure(8);
% surf(Tri);
% hold on;
image(Tri,'CDataMapping','scaled');
% hold off;
colorbar;