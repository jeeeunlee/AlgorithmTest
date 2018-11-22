function robot_satate=jet_update_kinematics(robot_satate, robot_param)

% BASE to left_foot
left_foot.q = robot_satate.q(3:8);
left_foot = forward_kinematics(left_foot, robot_param.left_foot);

% BASE to right_foot
right_foot.q = robot_satate.q(9:14);
right_foot = forward_kinematics(right_foot, robot_param.right_foot);

% Base to waist
waist.q = zeros(robot_param.waist.dof);
waist = forward_kinematics(waist, robot_param.waist);

% Waist to left_hand
left_hand.q = zeros(robot_param.left_hand.dof);
left_hand = forward_kinematics(left_hand, robot_param.left_hand);
% Base to left_hand
for i=1:size(left_hand.T,3)
    left_hand.T(:,:,i) = waist.T(:,:,end)*left_hand.T(:,:,i);
end

% Waist to right_hand
right_hand.q = zeros(robot_param.right_hand.dof);
right_hand = forward_kinematics(right_hand, robot_param.right_hand);
% Base to right_hand
for i=1:size(right_hand.T,3)
    right_hand.T(:,:,i) = waist.T(:,:,end)*right_hand.T(:,:,i);
end

% Waist to head
head.q = zeros(robot_param.head.dof);
head = forward_kinematics(head, robot_param.head);
% Base to head
for i=1:size(head.T,3)
    head.T(:,:,i) = waist.T(:,:,end)*head.T(:,:,i);
end

robot_satate.left_foot= left_foot;
robot_satate.right_foot= right_foot;
robot_satate.waist= waist;
robot_satate.left_hand= left_hand;
robot_satate.right_hand= right_hand;
robot_satate.head= head;

end