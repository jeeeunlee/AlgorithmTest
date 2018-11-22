function robot_state = forward_kinematics(robot_state, robot_param)

robot_state.T = zeros(4, 4, robot_param.dof + 1);
T = eye(4);
for i = 1 : robot_param.dof
  T = T * robot_param.M(:, :, i) * LargeSE3(robot_param.S(:, :, i) * robot_state.q(i));    
  robot_state.T(:, :, i) = T;
end

robot_state.T(:, :, robot_param.dof+1) = robot_state.T(:,:,robot_param.dof) * robot_param.tcp;


