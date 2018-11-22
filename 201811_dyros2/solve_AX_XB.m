function X = solve_AX_XB(A_m,B_m)
% X = 4X4 homogeneous matrix with (R,p)
% A_m : screw(6x1) X n, B_m = screw(6x1) X n

n = size(A_m,2);

M = zeros(3,3);C=[];d=[];I3=eye(3);
for i = 1:n
    wa = A_m(1:3,i);
    wb = B_m(1:3,i);
    
%     M = M + wa*wb';  
    M = M + wb*wa';
end

[U,S,V] = svd(M);

% R = (M'M)^-1/2 * M'
R = V*(diag(1./diag(S)))*V' * M';

for i = 1:n
    Ta = LargeSE3(A_m(1:6,i));
    Tb = LargeSE3(B_m(1:6,i));
    
    C = [C; I3-Ta(1:3,1:3)];
    d = [d; Ta(1:3,4)-R*Tb(1:3,4)];
end

p=(C'*C)\C'*d;

X = [R,p;0,0,0,1];
end