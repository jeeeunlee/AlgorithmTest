clear all; close all; clc;
addpath('LieGroupLibrary');
addpath('data');
addpath('KinLibrary');

%% load data
walking_data = load('walking_data.txt'); %  2561 X 14: 2-tick/step + 12-dirobm joint

q_data_upper_body = zeros(size(walking_data,1),4);
q_data_upper_body = 0.05*randn(size(q_data_upper_body));
% q_data_left = [walking_data(:,8:-1:3), q_data_upper_body];
% q_data_right = [walking_data(:,14:-1:9), q_data_upper_body];
% q_data_left = [0.1*randn(size(walking_data,1),6), ones(size(walking_data,1),1)];
% q_data_right = [0.1*randn(size(walking_data,1),6), ones(size(walking_data,1),1)];
q_data_left = [walking_data(:,8:-1:4),0.1*randn(size(walking_data,1),1),  q_data_upper_body];
q_data_right = [walking_data(:,14:-1:10),0.1*randn(size(walking_data,1),1),  q_data_upper_body];


walking_data_support = ones(size(walking_data,1),1);
walking_data_support(801:801+239) = 2;
walking_data_support(801+480:801+480+239) = 2;

%% load kinematics
load('jet'); %robot_param : left_foot, right_foot, waist, left_hand, right_hand
serial_param_left = get_serial_param2(robot_param,'left');
serial_param_right = get_serial_param2(robot_param,'right');
[serial_param_left_actual, serial_param_right_actual] = get_virtual_screw(serial_param_left,serial_param_right,robot_param.left_foot.dof);
serial_param_left.screw
serial_param_left_actual.screw
% serial_param_right.screw
% serial_param_right_actual.screw

global step_size;
left_i=[0];
right_i=[0];
iteration=0; i_start = 600; step_size = round(200/20);
while (iteration < 10000000 && i_start<1760)
    iteration = iteration+1;
   
    map = getMap(i_start, serial_param_left, serial_param_right, serial_param_left_actual, serial_param_right_actual,  walking_data_support, q_data_left, q_data_right);
    if(map.left==1)
        if(left_i(end)<i_start)
            left_i = [left_i, ]
        end
    else
    end
    J_m = map.J_m; y_m=map.y_m;
    delx = (transpose(J_m)*J_m)\transpose(J_m)*y_m ;
    % delx = pinv(transpose(J_m)*J_m)*transpose(J_m)*y_m ;
    
    if(norm(y_m)>1e-5 && norm(delx)>1e-5 && map.left==1)
        

        serial_param_left.screw(:,end) = LogSE3(InverseSE3(solve_AX_XB(map.A_m, map.B_m)));
        serial_param_left = update_robot_param_calib(serial_param_left, delx);
            
        
        display('left='); display([norm(y_m),norm(delx)]); display(serial_param_left.screw)
    elseif(norm(y_m)>1e-8 && map.right==1)
        serial_param_right = update_robot_param_calib(serial_param_right, delx);
        display('right='); display([norm(y_m),norm(delx)]); display(serial_param_right.screw)
    else
        i_start = map.i_end;
    end 
    
end
display('left=');
display(serial_param_left.screw)
display(serial_param_left_actual.screw)
display('right=');
display(serial_param_right.screw)
display(serial_param_right_actual.screw)