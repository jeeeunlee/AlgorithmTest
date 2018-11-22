function robot_param = add_screw_robot_param_link(robot_param)
% screw for POE
    robot_param.screw = zeros(6, robot_param.dof + 1);

    M_accum = eye(4);

    for i = 1 : robot_param.dof
        M_accum = M_accum * robot_param.M(:,:,i);    
        robot_param.screw(:,i) = Adj(M_accum, robot_param.S(:,:,i));

    end

    robot_param.screw(:,robot_param.dof+1) = LogSE3(M_accum * robot_param.tcp);    
end