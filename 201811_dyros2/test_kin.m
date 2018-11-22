function robot_param = test_kin(robot_param)

robot_param.screw = zeros(6, robot_param.dof + 1);
robot_param.screw2 = zeros(6, robot_param.dof + 1);


M_accum = eye(4);
for i = 1 : robot_param.dof
    M_accum = M_accum * robot_param.M(:,:,i);
    robot_param.screw(:,i) = Adj(M_accum, robot_param.S(:,:,i));
    
    robot_param.screw2(:,i) = robot_param.S(:,:,i);
    robot_param.screw2(4:6,i) = -skew(robot_param.S(:,:,i))*M_accum(1:3,4);
end

robot_param.screw(:,robot_param.dof+1) = LogSE3(robot_param.tcp);
robot_param.screw2(:,robot_param.dof+1) = LogSE3(robot_param.tcp);

end

