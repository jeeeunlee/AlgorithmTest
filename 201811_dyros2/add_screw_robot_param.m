function robot_param_whole = add_screw_robot_param(robot_param_whole)

robot_param_whole.left_foot = add_screw_robot_param_link(robot_param_whole.left_foot);
robot_param_whole.right_foot = add_screw_robot_param_link(robot_param_whole.right_foot);
robot_param_whole.waist = add_screw_robot_param_link(robot_param_whole.waist);
robot_param_whole.left_hand = add_screw_robot_param_link(robot_param_whole.left_hand);
robot_param_whole.right_hand = add_screw_robot_param_link(robot_param_whole.right_hand);
robot_param_whole.head = add_screw_robot_param_link(robot_param_whole.head);

end

