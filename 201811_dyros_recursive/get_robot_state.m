function robot_state = get_robot_state(robot_param)

robot_state.q = zeros(robot_param.dof,1);
robot_state.dq = zeros(robot_param.dof,1);
robot_state.ddq = zeros(robot_param.dof,1);
robot_state.torque = zeros(robot_param.dof,1);

