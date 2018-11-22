function J = getJ_origin(serial_param, serial_state_prev, serial_state)

dof = serial_param.dof;

J = zeros(6,6*dof+6);
% J = zeros(6,6*dof);
serial_state = forward_kinematics_POE(serial_state, serial_param);

for i=1:dof+1
% for i=1:dof
    Ai = getAi(serial_param.screw(:,i), serial_state.q(i));

    if(i==1)
        Ta = eye(4);
    else
        Ta = serial_state.T(:,:,i-1);
    end    
    J(:,(6*(i-1)+1):(6*i)) = Ad_T(Ta) * Ai;
end

