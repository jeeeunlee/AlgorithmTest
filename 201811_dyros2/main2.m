clear all; close all; clc;
addpath('LieGroupLibrary');
addpath('data');

load('jet'); %robot_param : left_foot, right_foot, waist, left_hand, right_hand
walking_data = load('walking_data.txt'); %  2561 X 14: 2-tick/step + 12-dirobm joint

walking_data_support = ones(size(walking_data,1),1);
walking_data_support(801:801+239) = 2;
walking_data_support(801+480:801+480+239) = 2;

serial_param_left = get_serial_param2(robot_param,'left');
serial_param_right = get_serial_param2(robot_param,'right');
[serial_param_left_actual, serial_param_right_actual] = get_virtual_screw(serial_param_left,serial_param_right,robot_param.left_foot.dof);
% serial_param_left_actual.screw
% serial_param_right_actual.screw

q_data_upper_body = zeros(size(walking_data,1),4);
q_data_upper_body = 0.0005*randn(size(q_data_upper_body));
q_data_left = [walking_data(:,8:-1:3), q_data_upper_body];
q_data_right = [walking_data(:,14:-1:9), q_data_upper_body];

%% PLOT

option.initialize = true;
option.display.arm=false;

frame.fixed_frame = 'base'; % base, left_foot, right_foot
frame.world_to_base = eye(4);

figure(1)
while true
    for i=600:30:1760
        cla reset;
                
        robot_state.q = q_data_left(i,:);
        robot_state = forward_kinematics(robot_state,serial_param_left);
        basemat = InverseSE3(robot_state.T(:,:,7));
        for i=1:serial_param_left.dof+1
            robot_state.T(:,:,i) = basemat * robot_state.T(:,:,i) ;
            T0 = basemat * eye(4);
        end
        p = [];
        p(1:3,1:serial_param_left.dof+1)=robot_state.T(1:3,4,:);
        p = [T0(1:3,4,:), p];
        plot3(p(1,:),p(2,:),p(3,:),'o-'); hold on;
        plot3(p(1,7),p(2,7),p(3,7),'.'); hold on;
        
        robot_state = forward_kinematics_POE(robot_state,serial_param_left);
        robot_state.T(:,:,end) = basemat * robot_state.T(:,:,end) ;
        pTCP = robot_state.T(1:3,4,end);
        plot3(pTCP(1),pTCP(2),pTCP(3),'d');
        
        robot_state.q = q_data_right(i,:);
        robot_state = forward_kinematics(robot_state,serial_param_right);
        basemat = InverseSE3(robot_state.T(:,:,7));
        for i=1:serial_param_right.dof+1
            robot_state.T(:,:,i) = basemat * robot_state.T(:,:,i);
            T0 =  basemat * eye(4);
        end
        p = [];
        p(1:3,1:serial_param_right.dof+1)=robot_state.T(1:3,4,:);
        p = [T0(1:3,4,:), p];
        plot3(p(1,:),p(2,:),p(3,:),'x-'); 
        plot3(p(1,7),p(2,7),p(3,7),'.'); hold on;
        
        robot_state = forward_kinematics_POE(robot_state,serial_param_right);
        robot_state.T(:,:,end) = basemat * robot_state.T(:,:,end) ;
        pTCP = robot_state.T(1:3,4,end);
        plot3(pTCP(1),pTCP(2),pTCP(3),'d');
        
        axis equal; grid on;
        
        pause(.1)        
    end
end