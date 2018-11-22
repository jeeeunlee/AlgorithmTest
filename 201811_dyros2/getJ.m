function J = getJ(serial_param, serial_state_prev, serial_state)

dof = serial_param.dof;

% J = zeros(6,6*dof+6);
J = zeros(6,6*dof);
serial_state = forward_kinematics_POE(serial_state, serial_param);
serial_state_prev = forward_kinematics_POE(serial_state_prev, serial_param);

for i=1:dof
    Ai = getAi(serial_param.screw(:,i), serial_state.q(i));
%     Bi = getBi(serial_param.screw(:,i), serial_state_prev.q(i));
    Bi = getAi(serial_param.screw(:,i), -serial_state_prev.q(i));
    if(i==1)
        Ta = InverseSE3(serial_state_prev.T(:,:,end));
    else
        Ta = InverseSE3(serial_state_prev.T(:,:,end))* serial_state.T(:,:,i-1);
    end
    
    Tb = InverseSE3(serial_state_prev.T(:,:,end))*serial_state_prev.T(:,:,i);
    
    J(:,(6*(i-1)+1):(6*i)) = Ad_T(Ta) * Ai + Ad_T(Tb) * Bi;
end

% Ta = InverseSE3(serial_state_prev.T(:,:,end))* serial_state.T(:,:,dof);
% Ai = getAi(serial_param.screw(:,end), 1);
% Bi = getBi(serial_param.screw(:,end), 1);
% 
% J(:,(6*dof+1):(6*dof+6)) = Ad_T(Ta) * Ai + Bi;