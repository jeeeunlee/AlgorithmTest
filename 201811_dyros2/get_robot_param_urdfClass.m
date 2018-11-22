function robot_param = get_robot_param_urdfClass(robotClass)

% left_foot, right_foot TCP
foot_tcp = eye(4);
foot_tcp(1:3,4)=[0,0,-0.075];

robot_param.dof = robotClass.NumBodies;
dof=robot_param.dof;

% Reduction ratio
% robot_param.transmission = zeros(1,dof); % reduction ratio
% RPM
% robot_param.max_rpm = [3500,3500,3500,3500,3500,3500]; % rpm
% MAX ACC
% robot_param.joint_max_acc = [15,15,15,15,15,15]; % rad/s2
% MAX VEL
% robot_param.joint_max_vel = 2*pi/SEC2MIN*robot_param.max_rpm./robot_param.transmission;
% MAX TORQUE
% ratio = 0.85;
% robot_param.max_torque = [662, 612, 204, 96, 70, 56] * ratio;

robot_param.min_limit = zeros(1,dof);
robot_param.max_limit = zeros(1,dof);
I_com = zeros(6,dof);
P_com = zeros(3,dof);
mass = zeros(1,dof);
robot_param.BodyNames = {};

for i=1:dof
    % LIMIT
    robot_param.min_limit(i) = robotClass.Bodies{i}.Joint.PositionLimits(1);
    robot_param.max_limit(i) = robotClass.Bodies{i}.Joint.PositionLimits(2);
    
    % MASS
    mass(i) = robotClass.Bodies{i}.Mass;
    
    % Link Name
    robot_param.BodyNames{i} = robotClass.BodyNames{i};
    robot_param.BaseName = robotClass.BaseName;    
    
    % Ixx, Iyy, Izz, Iyz, Ixz, Ixy
    I_com(:,i) = robotClass.Bodies{i}.Inertia;
    
    % centerofMass
    P_com(:,i) =robotClass.Bodies{i}.CenterOfMass;
end

index = 0;
%% LEFT FOOT
% Initial Configuration
robot_param.left_foot.dof = 6;
% JointToParentTransform
robot_param.left_foot.M = zeros(4,4,robot_param.left_foot.dof);
% Screw for each joint
robot_param.left_foot.S = zeros(6,1,robot_param.left_foot.dof);
for i = 1:robot_param.left_foot.dof
    % link i
    robot_param.left_foot.M(:,:,i) = robotClass.Bodies{index+i}.Joint.JointToParentTransform;
    robot_param.left_foot.S(1:3,:,i) = robotClass.Bodies{index+i}.Joint.JointAxis;
end

robot_param.left_foot.T_m = zeros(4,4,robot_param.left_foot.dof);
T_accum = eye(4);
for i = 1 : robot_param.left_foot.dof
    T_accum = T_accum * robot_param.left_foot.M(:,:,i);
    robot_param.left_foot.T_m(:,:,i) = T_accum;
end
robot_param.left_foot.tcp = foot_tcp;
index = index + robot_param.left_foot.dof;

%% Right FOOT
% Initial Configuration
robot_param.right_foot.dof = 6;
% JointToParentTransform
robot_param.right_foot.M = zeros(4,4,robot_param.right_foot.dof);
% Screw for each joint
robot_param.right_foot.S = zeros(6,1,robot_param.right_foot.dof);

for i = 1:robot_param.right_foot.dof
    % link i
    robot_param.right_foot.M(:,:,i) = robotClass.Bodies{index+i}.Joint.JointToParentTransform;
    robot_param.right_foot.S(1:3,:,i) = robotClass.Bodies{index+i}.Joint.JointAxis;
end

robot_param.right_foot.T_m = zeros(4,4,robot_param.right_foot.dof);
T_accum = eye(4);
for i = 1 : robot_param.right_foot.dof
    T_accum = T_accum * robot_param.right_foot.M(:,:,i);
    robot_param.right_foot.T_m(:,:,i) = T_accum;
end
robot_param.right_foot.tcp = foot_tcp;
index = index + robot_param.right_foot.dof;

%% waist
% Initial Configuration
robot_param.waist.dof = 2;
% JointToParentTransform
robot_param.waist.M = zeros(4,4,robot_param.waist.dof);
% Screw for each joint
robot_param.waist.S = zeros(6,1,robot_param.waist.dof);

for i = 1:robot_param.waist.dof
    % link i
    robot_param.waist.M(:,:,i) = robotClass.Bodies{index+i}.Joint.JointToParentTransform;
    robot_param.waist.S(1:3,:,i) = robotClass.Bodies{index+i}.Joint.JointAxis;
end

robot_param.waist.T_m = zeros(4,4,robot_param.waist.dof);
T_accum = eye(4);
for i = 1 : robot_param.waist.dof
    T_accum = T_accum * robot_param.waist.M(:,:,i);
    robot_param.waist.T_m(:,:,i) = T_accum;
end
robot_param.waist.tcp = eye(4);
index = index + robot_param.waist.dof;

%% LeftHand
% Initial Configuration
robot_param.left_hand.dof = 7;
% JointToParentTransform
robot_param.left_hand.M = zeros(4,4,robot_param.left_hand.dof);
% Screw for each joint
robot_param.left_hand.S = zeros(6,1,robot_param.left_hand.dof);

for i = 1:robot_param.left_hand.dof
    % link i
    robot_param.left_hand.M(:,:,i) = robotClass.Bodies{index+i}.Joint.JointToParentTransform;
    robot_param.left_hand.S(1:3,:,i) = robotClass.Bodies{index+i}.Joint.JointAxis;
end

robot_param.left_hand.T_m = zeros(4,4,robot_param.left_hand.dof);
T_accum = eye(4);
for i = 1 : robot_param.left_hand.dof
    T_accum = T_accum * robot_param.left_hand.M(:,:,i);
    robot_param.left_hand.T_m(:,:,i) = T_accum;
end
robot_param.left_hand.tcp = eye(4);
index = index + robot_param.left_hand.dof;

%% Right Hand
% Initial Configuration
robot_param.right_hand.dof = 7;
% JointToParentTransform
robot_param.right_hand.M = zeros(4,4,robot_param.right_hand.dof);
% Screw for each joint
robot_param.right_hand.S = zeros(6,1,robot_param.right_hand.dof);

for i = 1:robot_param.right_hand.dof
    % link i
    robot_param.right_hand.M(:,:,i) = robotClass.Bodies{index+i}.Joint.JointToParentTransform;
    robot_param.right_hand.S(1:3,:,i) = robotClass.Bodies{index+i}.Joint.JointAxis;
end

robot_param.right_hand.T_m = zeros(4,4,robot_param.right_hand.dof);
T_accum = eye(4);
for i = 1 : robot_param.right_hand.dof
    T_accum = T_accum * robot_param.right_hand.M(:,:,i);
    robot_param.right_hand.T_m(:,:,i) = T_accum;
end
robot_param.right_hand.tcp = eye(4);
index = index + robot_param.right_hand.dof;

%% Head : TODO
% Initial Configuration
% robot_param.right_hand.dof = 2;
% % JointToParentTransform
% robot_param.right_hand.M = zeros(4,4,robot_param.right_hand.dof);
% % Screw for each joint
% robot_param.right_hand.S = zeros(6,1,dof);
% 
% for i = index+1:index+robot_param.right_hand.dof
%     % link i
%     robot_param.right_hand.M(:,:,i) = robotClass.Bodies{i}.Joint.JointToParentTransform;
%     robot_param.right_hand.S(1:3,:,i) = robotClass.Bodies{1}.Joint.JointAxis;
% end
% 
% robot_param.right_hand.T_m = zeros(4,4,robot_param.right_hand.dof);
% T_accum = eye(4);
% for i = 1 : robot_param.right_hand.dof
%     T_accum = T_accum * robot_param.right_hand.M(:,:,i);
%     robot_param.right_hand.T_m(:,:,i) = T_accum;
% end
% index = index + robot_param.right_hand.dof;


% % Geleralized Inertia for each link
% robot_param.J = zeros(6,6,dof);
% % center of mass parameters
% J_com = zeros(6,6,dof);
% T_com = zeros(4,4,dof); 
% for i = 1 : dof
%     J_com(1:3,1:3,i) = [ I_com(1,i) I_com(6,i) I_com(5,i);
%         I_com(6,i) I_com(2,i) I_com(4,i);
%         I_com(5,i) I_com(4,i) I_com(3,i)];
%     J_com(4:6,4:6,i) = mass(i)*eye(3);
%     T_com(1:3,4,i) = P_com(:,i);
%     
%     % TODO: Rotation matrix must be set.
%     T_com(1:3,1:3,i) = eye(3);
%     % T_com(1:3,1:3,i) = R_com(:,:,i);
%     
%     T_com_to_frame = InverseSE3(T_com(:,:,i)) * T_m(:,:,i);
%     Ad_i = Ad_T(T_com_to_frame);
%     
%     robot_param.J(:,:,i) = Ad_i' * J_com(:,:,i) * Ad_i;    
% end

end