function [o_ddq dV0] = FloatingSerialForwardDynamicsImpulse(i_q, i_dq,v,i_tau,dt,gravity)
% Lie Group Forward Dynamics Algorithm for Serial Open Chain Systems
% ddq = SerialForwardDynamics(q, dq, tau)
% n : # of links
% q, dq, tau, ddq : n*1
% Syungkwon Ra (dearLenin@gmail.com)

% Initialize Parameters of Systems
global g_n g_V0 g_dV0 g_S g_M g_J g_ft g_Fext;

% variables
i = 0;                      % iterator
aJ = zeros(6, 6, g_n);      % artigulated inertia
b = zeros(6, 1, g_n);       % dse(3), bias force
V = zeros(6, 1, g_n);       % se(3), V(:, :, i) = V_i
dV = zeros(6, 1, g_n);      % se(3), dV(:, :, i) = dV_i
invf = zeros(4, 4, g_n);    % SE(3), invf(:, :, i) = inv(f_i-1_i)
omega = zeros(1, g_n);      % S' * aJ * S;
c = zeros(6, 1, g_n);       % b + aJ * ad_V_Sq
ddq = zeros(g_n-1);         % angular acceleration

%%%%%%%%%%%%%%% Lie Algorithms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization
% V(:, :, 1) = g_V0;
% dV(:, :, 1) = g_dV0;
aJ(:, :, g_n) = g_J(:, :, g_n);
%% new Initialization for floating body
% Initialization


% Forward recursion : for i = 1 to n
for i = 2 : g_n
    f = g_M(:, :, i-1) * LargeSE3(g_S(:, :, i-1) * i_q(i-1));
    invf(:, :, i) = InverseSE3(f);
end

aJ(:, :, g_n) = g_J(:, :, g_n);
% Backward recursion : for i = n-1 to 1 // articulated inertia update
for i = g_n-1 : -1 : 1
    omega(:, i+1) = g_S(:, :, i)' * aJ(:, :, i+1) * g_S(:, :, i);
    aJ(:, :, i) = g_J(:, :, i) + Transform(invf(:, :, i+1), aJ(:, :, i+1) - (aJ(:, :, i+1) * g_S(:, :, i) * g_S(:, :, i)' * aJ(:, :, i+1)) / omega(:, i+1));
end



% if g_ft(4, 4, g_n) == 1
    b(:, :, g_n) =  - dAdj(InverseSE3(g_ft(:, :, g_n)), g_Fext(:, :, g_n));
% end
omega(:, g_n) = g_S(:, :, g_n-1)' * aJ(:, :, g_n) * g_S(:, :, g_n-1);

% Backward recursion : for i = n-1 to 1
for i = g_n-1 : -1 : 2
    b(:, :, i) = dAdj(invf(:, :, i+1), b(:,:,i+1) - aJ(:, :, i+1) * g_S(:, :, i) * g_S(:,:,i)' * b(:,:,i+1) / omega(:, i+1));
%     if g_ft(4, 4, i) == 1
        b(:, :, i) = b(:, :, i) - dAdj(InverseSE3(g_ft(:, :, i)), g_Fext(:, :, i));
%     end
    omega(:, i) = g_S(:, :, i-1)' * aJ(:, :, i) * g_S(:, :, i-1);
end

%% Evaluate Accle. of base body in floating body
b(:, :, 1) = dAdj(invf(:, :, 2), b(:, :, 2) - aJ(:, :, 2) * g_S(:, :, 1) * g_S(:, :, 1)'*b(:, :, 2)  / omega(:, 2));
% if g_ft(4, 4, 1) == 1
    b(:, :, 1) = b(:, :, 1) - dAdj(InverseSE3(g_ft(:, :, 1)), g_Fext(:, :, 1));
% end

 


dV(:,:,1)=-inv(aJ(:,:,1))*b(:,:,1);


% Forward recursion : for i = 1 to n
for i = 2 : g_n
    ddq(i-1) = (- g_S(:, :, i-1)' * (aJ(:, :, i) * Adj(invf(:, :, i), dV(:, :, i-1)) + b(:, :, i)))/ omega(:, i);
    dV(:, :, i) = g_S(:, :, i-1) * ddq(i-1) + Adj(invf(:, :, i), dV(:, :, i-1));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

o_ddq =  ddq(1);
for i = 2:g_n-1
    o_ddq = [o_ddq; ddq(i)];
end
dV0=dV(:,:,1);
