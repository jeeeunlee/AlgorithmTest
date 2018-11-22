function [robot_state,frame] = update_robot_base(robot_state, frame)

% update fixed_frame & world_to_base 

% robot_satate.left_foot= left_foot;
% robot_satate.right_foot= right_foot;
% robot_satate.waist= waist;
% robot_satate.left_hand= left_hand;
% robot_satate.right_hand= right_hand;
% robot_satate.head= head;

base_to_left_foot = robot_state.left_foot.T(:,:,end);
base_to_right_foot = robot_state.right_foot.T(:,:,end);

if( abs(base_to_left_foot(3,4)) > abs(base_to_right_foot(3,4)) )
    fixed_frame = 'left_foot';
else
    fixed_frame = 'right_foot';
end

switch(frame.fixed_frame)
    case 'left_foot'
        frame.world_to_base = frame.world_to_base * (frame.base_to_left_foot) * InverseSE3(base_to_left_foot);
    case 'right_foot'
        frame.world_to_base = frame.world_to_base * (frame.base_to_right_foot) * InverseSE3(base_to_right_foot);
end

for i=1:size(robot_state.left_foot.T,3)
    robot_state.left_foot.T(:,:,i) = frame.world_to_base*robot_state.left_foot.T(:,:,i);
end
for i=1:size(robot_state.right_foot.T,3)
    robot_state.right_foot.T(:,:,i) = frame.world_to_base*robot_state.right_foot.T(:,:,i);
end
for i=1:size(robot_state.waist.T,3)
    robot_state.waist.T(:,:,i) = frame.world_to_base*robot_state.waist.T(:,:,i);
end
for i=1:size(robot_state.left_hand.T,3)
    robot_state.left_hand.T(:,:,i) = frame.world_to_base*robot_state.left_hand.T(:,:,i);
end
for i=1:size(robot_state.right_hand.T,3)
    robot_state.right_hand.T(:,:,i) = frame.world_to_base*robot_state.right_hand.T(:,:,i);
end
for i=1:size(robot_state.head.T,3)
    robot_state.head.T(:,:,i) = frame.world_to_base*robot_state.head.T(:,:,i);
end

switch(frame.fixed_frame)
    case 'left_foot'
        robot_state.fixed_frame = robot_state.left_foot.T(:,:,end);
    case 'right_foot'
        robot_state.fixed_frame = robot_state.right_foot.T(:,:,end);
    otherwise
        robot_state.fixed_frame = robot_state.right_foot.T(:,:,end);
end

% update
frame.base_to_left_foot = base_to_left_foot;
frame.base_to_right_foot = base_to_right_foot;
frame.fixed_frame = fixed_frame;

robot_state.base = frame.world_to_base;


end
