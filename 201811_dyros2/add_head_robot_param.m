function robot_param = add_head_robot_param(robot_param)

head_tcp=eye(4);
head_tcp(1:3,4)=[0.05;0;0.1];

%% Head
% Initial Configuration
robot_param.BodyNames{robot_param.dof+1} = 'Head_Roll_Link';
robot_param.BodyNames{robot_param.dof+2} = 'Head_Yaw_Link';
head.dof = 2;

head.min_limit = [-3.14,-0.3];
head.max_limit = [3.14,0.3];

% JointToParentTransform
head.M = zeros(4,4,head.dof);
% Screw for each joint
head.S = zeros(6,1,head.dof);

head.M(:,:,1) = [[eye(3),[0;0;0.3]];[0,0,0,1]];
head.S(1:3,:,1) = [0;0;1];

head.M(:,:,2) = eye(4);
head.S(1:3,:,2) = [0;1;0];

head.T_m = zeros(4,4,head.dof);
T_accum = eye(4);
for i = 1 : head.dof
    T_accum = T_accum * head.M(:,:,i);
    head.T_m(:,:,i) = T_accum;
end
head.tcp = head_tcp;


robot_param.head = head;

robot_param.min_limit = [robot_param.min_limit, head.min_limit];
robot_param.max_limit = [robot_param.max_limit, head.max_limit];

robot_param.dof = robot_param.dof + robot_param.head.dof;

end