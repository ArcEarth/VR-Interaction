function [ P ] = perms( V, m )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


end

function P = permsr(V, k)
% subfunction to help with recursion

V = V(:).'; % Make sure V is a row vector
n = length(V);
if n <= 1
    P = V; 
    return; 
end

q = permsr(1:n-1 , k);  % recursive calls
m = size(q,1);
P = zeros(n*m,n);
P(1:m,:) = [n*ones(m,1) q];

for i = n-1:-1:1,
   t = q;
   t(t == i) = n;
   P((n-i)*m+1:(n-i+1)*m,:) = [i*ones(m,1) t]; % assign the next m
                                               % rows in P.
end

P = V(P);
end
