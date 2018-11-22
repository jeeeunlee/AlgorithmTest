function robot_param = update_robot_param_calib(robot_param, delx)

global alpha
alpha = 1;

dof = robot_param.dof;

for i=1:dof
    robot_param.screw(:,i) = update_screw(robot_param.screw(:,i),delx((6*(i-1)+1):(6*i)));
end

if((length(delx)/6) > dof)
    robot_param.screw(:,dof+1) = robot_param.screw(:,dof+1) + alpha*delx((6*(dof)+1):(6*dof+6));    
end