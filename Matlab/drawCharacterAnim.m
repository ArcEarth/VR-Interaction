uj = 5; 
cj = 8;

Ku = Xr(:,:,uj);
figure(3);
plot(1:T,Ku(:,1),1:T,Ku(:,2),1:T,Ku(:,3),'LineSmoothing','on');
title(sprintf('User Joint %d Motion (X-B,Y-G,Z-R)',uj));

phi = 13;%Phi(uj,cj);
% for phi = 1 : T
    Kc = Y(phi:T+phi-1,1:3,cj);
    figure(2);
    plot(1:T,Kc(:,1),1:T,Kc(:,2),1:T,Kc(:,3),'LineSmoothing','on');
    title(sprintf('Character Joint "%s" Motion (X-B,Y-G,Z-R)',bonenames{cj}));

    [A,B,r,U,V]=canoncorr(Ku,Kc);
    uC = repmat(mean(Kc),size(Kc,1),1);
    Vr = U/B + uC;
    figure(12);
    plot(1:T,Vr(:,1),1:T,Vr(:,2),1:T,Vr(:,3),'LineSmoothing','on');
% end
figure(5);
plot(1:C,Ec);
plot(1:C,Ec,'.-','LineSmoothing','on');
title(sprintf('Character Joint Energy at all Frequency'));

figure(6);
plot(1:J,Ej,'.-','LineSmoothing','on');
title(sprintf('User Joint Energy'));

% figure(1);
% plot(1:T,ErrPhi(:,uj,cj),'LineSmoothing','on');
% title('Reconstruction error by phase');

figure(4);
plot(1:C,R(uj,:),'g--.',1:C,Rn(uj,:),'b-o','LineSmoothing','on');
title('Binding Correlation (Green - Before Energy Normalization, Blue After Normalization)');
Idx = find(Rn(uj,:) > 0);
for i = Idx
    text(i+0.5,Rn(uj,i)+0.005,sprintf('%d: %s',i,bonenames{i}));
end

figure(9);
Re = Rn;
% Re(Re > 0.09) = 1;
image(Re,'CDataMapping','scaled');
title('Motion Correlation');
colorbar;

figure(10);
Re = diag(Ej) * ones(J,C) * diag(Ec);
% Re(Re > 0.15) = 1;
image(Re,'CDataMapping','scaled');
title('Enengy cross');
colorbar;

%%
Juk = [2 5 8 10 13 ]';
% Jck = [ 8 12 14 19 24 29 34 39 44 49 ]';
Jck = [ 8 12 17 22 34 39 44 49 55 ]';
Re = R(Juk,Jck);
figure(11);
image(Re,'CDataMapping','scaled');
title('Selected Motion Correlation');
colorbar;

Kf = 1 + mod(floor(rand(1)),T);
Xkf = reshape(sum(Xr(:,:,Juk)),[3, size(Juk,1)]);
Ykf = reshape(sum(Y(:,1:3,Jck)),[3 , size(Jck,1)]);
% Ykf = Ykf - repmat(Y(1,:,1),1,size(Jck));

Xkf = Xkf ./ repmat(sqrt(dot(Xkf,Xkf)),3,1);
Ykf = Ykf ./ repmat(sqrt(dot(Ykf,Ykf)),3,1);
% Ykf(1,:) = - Ykf(1,:);
Rs = Xkf' * Ykf;

figure(14);
image(Rs,'CDataMapping','scaled');
title('Selected Spatial * Motion Correlation');
colorbar;

%%

[val, m1, m2] = bipartite_matching(Rs.*Re);
Binds = cell(length(m1),2);
Binds(:,1) = userbonenames(Juk(m1));
Binds(:,2) = bonenames(Jck(m2));
Binds

%%
n = size(Xkf,2);
m = length(Jck);
idx = ~eye(n,n);
idx = triu(idx);
idx = repmat(reshape(idx, [1 n n]),3 , 1 ,1);

Xkf = (Xkf - repmat(min(Xkf,[],2),1,n)) ./ repmat(max(Xkf,[],2) - min(Xkf,[],2),1,n);
Xdif = reshape(repmat(Xkf,n,1),[3 n n]) - reshape(repmat(Xkf,[1 n]),[3 n n]);
Lx = reshape(sum(Xdif,2), [3 n]);

Xpif = reshape(Xdif(idx),[3 (n-1)*n/2]);
Xpif = Xpif ./ repmat(sqrt(dot(Xpif,Xpif)),3,1);

Err = inf;
arrbase = 1: m;
arr = 1 : n;
Rsb = zeros(m*(m-1)*(m-2)*(m-3),1);
id = 1;
bestLK = 0;

for arr1 = 1 : m
    temp = arrbase(1);
    arrbase(1) = arrbase(arr1);
    arrbase(arr1) = temp; 
    for arr2 = 2 : m
        temp = arrbase(2);
        arrbase(2) = arrbase(arr2);
        arrbase(arr2) = temp; 
        for arr3 = 3 : m
            temp = arrbase(3);
            arrbase(3) = arrbase(arr3);
            arrbase(arr3) = temp; 
            for arr4 = 4 : m
                temp = arrbase(4);
                arrbase(4) = arrbase(arr4);
                arrbase(arr4) = temp; 
                arr = arrbase(1:n);

                Ymap = Ykf(:,arr);
                Ymap = (Ymap - repmat(min(Ymap,[],2),1,n)) ./ repmat(max(Ymap,[],2) - min(Ymap,[],2),1,n);
                Ydif = reshape(repmat(Ymap,n,1),[3 n n]) - reshape(repmat(Ymap,[1 n]),[3 n n]);
                Ly = reshape(sum(Ydif,2), [3 n]);
                Ypif = reshape(Ydif(idx),[3 (n-1)*n/2]);
                Ypif = Ypif ./ repmat(sqrt(dot(Ypif,Ypif)),3,1);
                
                Ly = Ly - Lx;
                err = sum(sqrt(dot(Ly,Ly)),2);
                
%                 theta = Xpif' * Ypif;
%                 theta = acos(theta);
%                 err = sum(sum(theta,2),1);
                % spatial err

%                 if (err < Err)
%                     bestArr = arr;
%                     Err = err;
%                 end
                
                Rsp = 1;% exp(-err*err/(30*30));

%                 PiarIdx = (Jck(arr)-1) * J + Juk;
%                 Rmt = prod(Rn(PiarIdx));
                PiarIdx2 = (arr-1) * n + (1:n);
                Rmt = sum(Rs(PiarIdx2) .* Re(PiarIdx2));
                
                likilhood = Rmt * Rsp;
                if (likilhood > bestLK)
                    bestLK = likilhood;
                    bestArr = arr;
                end
                 
                Rsb(id) = likilhood;
                id = id +1;
                
                temp = arrbase(4);
                arrbase(4) = arrbase(arr4);
                arrbase(arr4) = temp;                 
            end
            temp = arrbase(3);
            arrbase(3) = arrbase(arr3);
            arrbase(arr3) = temp; 
        end
        temp = arrbase(2);
        arrbase(2) = arrbase(arr2);
        arrbase(arr2) = temp;         
    end
    temp = arrbase(1);
    arrbase(1) = arrbase(arr1);
    arrbase(arr1) = temp;     
end

Binds = bonenames(Jck(bestArr));
figure(13);
plot(1:length(Rsb),Rsb,'LineSmoothing','on');
title('Binding-Spatial Error');