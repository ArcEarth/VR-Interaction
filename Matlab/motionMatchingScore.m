%% Reading and preprocessing data
%X = csvread('user.punch.csv');
X = csvread('user.attack.csv');
% X = csvread('user.walk.csv');
%X = csvread('user.horse.run.csv');
%X = csvread('user.spider.walk.csv');
Y = csvread('spider.attack.csv');
% Y = csvread('spider.walk.csv');
%Y = csvread('horse.run.csv');
%Y = csvread('Horse_Walk.csv');

%BoneNames = csvread('bone_names.csv');

Yd = 6;
C = floor(size(Y,2) / Yd);
Y = Y(:,1:size(Y,2)-1);
Y = reshape(Y,[size(Y,1) Yd C]);
Yd = 3;
% Y =  reshape(Y(:,4:6,:),[size(Y,1) Yd C]);
Y =  reshape(Y(:,1:3,:),[size(Y,1) Yd C]);

Jk = 15;
Xd = 3;
X = X(91:size(X,1)-90,1:min((Jk-1)*Xd,size(X,2))); % remove beginning and ending
if (size(X,1) > 450)
    X = resample(X,450,size(X,1));
end

N = size(X,1); %Observations
M = size(X,2);
J = floor(M/3);
%% Filtering & Frequency domin state
F = fft(X);
E = F .* conj(F);
Ea = sum(E,2);
Ej = reshape(E,[N,3,J]);
Ej = sum(Ej,2);
Ej = reshape(Ej,[N,J]);

figure(4);
plot(4:30,Ea(4:30),'LineSmoothing','on'); %Enger spectrum
title('Overall Energy spectrum');

[~,peek] = max(Ea(4:30));
peek = peek + 2; %freq

T = floor(N / peek);

Eth = 0.0001;
G = 0 : N-1;
G = G' ./ N * 12;
G = - (G .* G);
G = exp(G);
F(E <= Eth) = 0;
F = F .* repmat(G,1,M);

Xr = real(ifft(F));
Xs = Xr;

Ej = [ 0.25 0.5 0.25] * Ej(peek : peek + 2,:);
Ej = sqrt(Ej)';
Ej = Ej ./ max(Ej);
Ej = sqrt(Ej);

% figure(1);
% plot(1:J, Ej, '-*');
% title('Joint Energy at peek base frequency');

N = T * peek;
Ne = T * (peek+1);

Xr = Xr (1:N,:);
Xr = reshape(Xr,[T,peek,M]);
Xr = sum(Xr,2);
Xr = reshape(Xr,[T,3,J]);
Xr(:,2,:) = -Xr(:,2,:); % I mess up the order of minus in outporting c++ program

%% Resample Y and state F-Domin 
% figure(5);
% plot(1:size(Y,1),Y(:,36));
Y = resample(repmat(Y,3,1),T,size(Y,1));
Y = Y(T+1:2*T,:);
% figure(6);
% plot(1:size(Y,1),Y(:,36));

% Y = Xr;
% Yd = Xd;
% C = J;

Y = reshape(Y,[T,Yd,C]);

Fy = fft(Y);
Ec = real(Fy .* conj(Fy));
Ec = reshape(sum(Ec,2),[size(Ec,1) size(Ec,3)]);
Ec = sqrt(sum(Ec(2:size(Ec,1),:))');
Ec = Ec ./ max(Ec);
Ec = sqrt(Ec);

% Xr = Y;
% J = C;
% Xd = Yd;
% Ej = Ec;

Y = repmat(Y,2,1,1);
%% Correlation calculation
R = zeros(J,C);
Rm = zeros(T,J,C);
ErrPhi = inf(T,J,C);
Phi = zeros(J,C);
for c = 1 : C
    for i = 1 : J
        K = Xr(:,:,i);
        Err = inf(1);
        for phi = 1 : T
            % Y shifted
            Ys = Y(phi:phi+T-1,:,c);
            try
                [A,B,r,U,V] = canoncorr(K,Ys);
                uC = repmat(mean(Ys),size(Ys,1),1);
                Vr = U/B + uC;
                Tr = A/B;
                cash = Tr*Tr';
                
                err = Vr - Ys; %real(ifft(Vr)) - Y(phi:phi+T-1,:,c);
                err = err .* err;
                err = sqrt(sum(sum(err),2)); % rebuild err            
                r = min(r);
            catch
                r = 0;
                err = inf;
            end
            ErrPhi(phi,i,c) = err;
            Rm(phi,i,c) = r;
            R(i,c) = max(r,R(i,c));
            if (err < Err)
                Phi(i,c) = phi;
                Err = err;
            end
       end
    end
end

% Y = repmat(Y,2,1,1);
% Y = reshape(Y,[2*T,Yd*C]);
% [~,T22,e] = qr(Y,0);
% rankY = sum(abs(diag(T22)) > eps(abs(T22(1)))*max(2*T,Yd*C));
% e = e(1:rankY);
% 
% R = zeros(J,C);
% Phi = zeros(J,C);
% Dim = zeros(J,C);
% for i = 1 : J
%     K = Xr(:,:,i);
%     for j = 1 : T
%         % Y shifted
%         Ys = Y(j:j+T-1,e);
%         try
%             [A,B,r] = canoncorr(K,Ys);
%         catch
%             r = 0;
%         end
%         Ry = ones(Yd,C);
%         Ry(e) = r;
%         rIdx = r > R(i,:);
%         Phi(i,rIdx) = j;
%         R(i,rIdx) = r(rIdx);
%     end
% end

r = 1./max(R,[],1)';
Rn = diag(Ej) * R * diag(r .* Ec);
% figure(1);
% imshow([Rn;0.5*ones(2,C);diag(Ej) * R;0.5*ones(2,C);R]);
% title('Binding Correclation : X - CharacterJoint x Y - HumanJoint');

drawCharacterAnim;

% figure(3);
% plot(1:J,R .* Ej, 1 : J, R);
% title('Joint Motion-Correlation * Enengy');
% 
% [~,Jbind] = max(R .* Ej);
% 
% K = Xr(:,:,Jbind);
% figure(2);
% plot(1:T,K(:,1),1:T,K(:,2),1:T,K(:,3));
% title('Binding Joint Motion (X-R,Y-G,B-Z)');

%imshow(R);