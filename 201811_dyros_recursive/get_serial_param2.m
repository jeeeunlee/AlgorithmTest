function serial_robot_param = get_serial_param2(robot_param, type)

% serial_robot_param -> foot, base, waist, head
% update serial_robot_param: dof, M, S, screw

switch type
    case 'left' % left_foot, base, waist, head
        robot_param.support_foot = robot_param.left_foot;
    case 'right' % right_foot, base, waist, head
        robot_param.support_foot = robot_param.right_foot;
end

%% update dof 
dof = robot_param.support_foot.dof;
serial_robot_param.dof = dof;

%% update S
S = zeros(6,1,dof);
for i=1:robot_param.support_foot.dof
    S(:,:,i) = -robot_param.support_foot.S(:,:,robot_param.support_foot.dof-i+1);
end
serial_robot_param.S = S;

%% update M, tcp
M = zeros(4,4,dof);

M(:,:,1) = InverseSE3(robot_param.support_foot.tcp);
for i=2:robot_param.support_foot.dof
   M(:,:,i) = InverseSE3(robot_param.support_foot.M(:,:,robot_param.support_foot.dof+2-i));
end

serial_robot_param.M = M;

M_accum = InverseSE3(robot_param.support_foot.M(:,:,1));
for i=1:robot_param.waist.dof
    M_accum = M_accum*robot_param.waist.M(:,:,i);
end
for i=1:robot_param.head.dof
    M_accum = M_accum* robot_param.head.M(:,:,i);
end

serial_robot_param.tcp = M_accum*robot_param.head.tcp * LargeSE3([0,0,30*pi/180,0,0,0]) ;

%% update serial screw
serial_robot_param = add_screw_robot_param_link(serial_robot_param);

end