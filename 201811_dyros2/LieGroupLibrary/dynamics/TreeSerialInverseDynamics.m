function o_tau = TreeSerialInverseDynamics(i_q, i_dq, i_ddq)
% Lie Group Inverse Dynamics Algorithm for Serial Open Chain Systems
% tau = TreeSerialInverseDynamics(q, dq, ddq)
% n : # of links
% q, dq, ddq, tau : n*1
% AyoungKim (siriuz@gmail.com)
% Beobkyun ver. + Juyoung ver.

% Initialize Parameters of Systems
global g_n g_V0 g_dV0 g_S g_M g_J g_ft g_Fext g_mu g_lambda;
%SystemSpecification;

% variables
i = 0;                      % iterator
invf = zeros(4, 4, g_n);    % SE(3), invf(:, :, i) = inv(f_i-1_i)
V = zeros(6, 1, g_n);       % se(3),  V(:, :, i) = V_i
dV = zeros(6, 1, g_n);      % se(3), dV(:, :, i) = dV_i
F = zeros(6, 1, g_n);       % dse(3), F(:, :, i) = F_i
tau = zeros(g_n-1);         % torque

%%%%%%%%%%%%%%% Lie Algorithms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization
V(:, :, 1) = g_V0;
dV(:, :, 1) = g_dV0;
mu = g_mu; lambda = g_lambda;

% Forward recursion : for i = 1 to n
for i = 2 : g_n
    invf(:, :, i) = inv(g_M(:, :, i-1) * LargeSE3(g_S(:, :, i-1) * i_q(i-1)));
%    invf = inv(f(:, :, i));
    V(:, :, i) = Adj( invf(:, :, i), V(:, :, lambda(i-1)+1)) + g_S(:, :, i-1) * i_dq(i-1);
    dV(:, :, i) = g_S(:, :, i-1) * i_ddq(i-1) + Adj(invf(:, :, i), dV(:, :, lambda(i-1)+1)) + ad(V(:, :, i), g_S(:, :, i-1) * i_dq(i-1));
end

% Backward recursion : for i = n to 1
for i = g_n : -1 : 2
    F(:, :, i) = g_J(:, :, i) * dV(:, :, i) - dad(V(:, :, i), g_J(:, :, i) * V(:, :, i));
    for j=1:3;
        if mu(i-1,j) ~= 0
            F(:, :, i) = F(:, :, i) + dAdj(invf(:, :, mu(i-1,j)+1), F(:, :, mu(i-1,j)+1));
        end
    end
 
    if g_ft(4, 4) == 1
        F(:, :, i) = F(:, :, i) - dAdj(inv(g_ft(:, :, i)), g_Fext(:, :, i));
    end
    tau(i-1) = g_S(:, :, i-1)' * F(:, :, i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

o_tau = tau(1);
for i = 2:g_n-1
    o_tau = [o_tau; tau(i)];
end
