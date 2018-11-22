clear all; close all; clc;
addpath('LieGroupLibrary');
addpath('data');
% [robot_param, robotClass] = get_robot_param_urdf('dyros_jet_robot.urdf');
% save('jetUrdfClass', 'robotClass');
% load('jetUrdfClass');
% robot_param = get_robot_param_urdfClass(robotClass);
% robot_param = add_head_robot_param(robot_param);
% save('jet');

load('jet'); %robot_param : left_foot, right_foot, waist, left_hand, right_hand, head
robot_param = add_screw_robot_param(robot_param);
%% PLOT
walking_data = load('walking_data.txt'); % 2561 X 14: 2-tick/step + 12-dirobm joint

option.initialize = true;
option.display.arm=false;

frame.fixed_frame = 'base'; % base, left_foot, right_foot
frame.world_to_base = eye(4);

while true
    for i=600:30:1760
        robot_state.q = walking_data(i,:);
        robot_state = jet_update_kinematics(robot_state, robot_param);
        
        [robot_state, frame] = update_robot_base(robot_state, frame);
 
        jet_plot3d(robot_state, option);
        pause(.005)
        option.initialize = false;
        % display(sprintf('%d',i));
    end
end