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
% serial_param_left.screw
% serial_param_left_actual.screw
% serial_param_right.screw
% serial_param_right_actual.screw
serial_param_left0 = serial_param_left;
serial_param_right0 = serial_param_right;

q_data_upper_body = zeros(size(walking_data,1),4);
q_data_upper_body = 0.05*randn(size(q_data_upper_body));
% q_data_left = [walking_data(:,8:-1:3), q_data_upper_body];
% q_data_right = [walking_data(:,14:-1:9), q_data_upper_body];
q_data_left = [0.2*randn(size(walking_data,1),6), ones(size(walking_data,1),1)];
q_data_right = [0.2*randn(size(walking_data,1),6), ones(size(walking_data,1),1)];

step_size = 6;
J_m=[]; y_m=[]; A_m=[]; B_m=[];
% for i=600+step_size:step_size:1760
i=600;
iteration=0;
i_start = i;
b_update_screw=false;
while (iteration < 100000 && i<1760)
    iteration = iteration+1;
    i = i+step_size;
    % J = getJ(serial_param, serial_state, serial_state_prev)   
    
    if(walking_data_support(i-step_size)==1 && walking_data_support(i)==1) %:left support
        serial_state_prev.q = q_data_left(i-step_size,:);
        serial_state.q = q_data_left(i,:);
        J = getJ(serial_param_left,serial_state_prev,serial_state);
        
        serial_state_prev = forward_kinematics_POE(serial_state_prev, serial_param_left);
        serial_state = forward_kinematics_POE(serial_state, serial_param_left);
        delg = InverseSE3(serial_state_prev.T(:,:,end)) * serial_state.T(:,:,end);
        delb = InverseSE3(serial_state_prev.T(:,:,end-1)) * serial_state.T(:,:,end-1);
        
        serial_state_prev = forward_kinematics_POE(serial_state_prev, serial_param_left_actual);
        serial_state = forward_kinematics_POE(serial_state, serial_param_left_actual);
        delg_actual = InverseSE3(serial_state_prev.T(:,:,end)) * serial_state.T(:,:,end) * errorSE3(0.00);
        
        y = LogSE3(delg_actual*InverseSE3(delg));
        
    elseif(walking_data_support(i-step_size)==2 && walking_data_support(i)==2) %:right support
        serial_state_prev.q = q_data_right(i-step_size,:);
        serial_state.q = q_data_right(i,:);
        J = getJ(serial_param_right,serial_state_prev,serial_state);
        
        serial_state_prev = forward_kinematics_POE(serial_state_prev, serial_param_right);
        serial_state = forward_kinematics_POE(serial_state, serial_param_right);
        delg = InverseSE3(serial_state_prev.T(:,:,end)) * serial_state.T(:,:,end);
        delb = InverseSE3(serial_state_prev.T(:,:,end-1)) * serial_state.T(:,:,end-1);
        
        serial_state_prev = forward_kinematics_POE(serial_state_prev, serial_param_right_actual);
        serial_state = forward_kinematics_POE(serial_state, serial_param_right_actual);
        delg_actual = InverseSE3(serial_state_prev.T(:,:,end)) * serial_state.T(:,:,end) * errorSE3(0.00) ;
        
        y = LogSE3(delg_actual*InverseSE3(delg));
    else
        delx = (transpose(J_m)*J_m)\transpose(J_m)*y_m ;
%         delx = pinv(transpose(J_m)*J_m)*transpose(J_m)*y_m ;
    
        if((norm(y_m)>1e-8 || norm(delx)>1e-8) && walking_data_support(i-step_size)==1)
%             if(norm(y_m)<1e-3)
            
%             end
%             if(b_update_screw)
%                 serial_param_left.screw(:,end) = LogSE3(InverseSE3(solve_AX_XB(A_m,B_m)));
%                 b_update_screw = false;
%             else
                serial_param_left = update_robot_param_calib(serial_param_left, delx);
%                 serial_param_left.screw(:,end) = LogSE3(InverseSE3(solve_AX_XB(A_m,B_m)));
%                 b_update_screw = true;
%             end
            display('left='); display([norm(y_m),norm(delx)]); display(serial_param_left.screw)
            i=i_start;
        elseif((norm(y_m)>1e-8 || norm(delx)>1e-8) && walking_data_support(i-step_size)==2)
            if(b_update_screw)
                serial_param_right.screw(:,end) = LogSE3(InverseSE3(solve_AX_XB(A_m,B_m)));
                b_update_screw = false;
            else
                serial_param_right = update_robot_param_calib(serial_param_right, delx);
                serial_param_right.screw(:,end) = LogSE3(InverseSE3(solve_AX_XB(A_m,B_m)));
                b_update_screw = true;
            end
            display('right='); display([norm(y_m),norm(delx)]); display(serial_param_right.screw)
            i=i_start;
        elseif(walking_data_support(i-step_size)==1)            
            display('left='); display([norm(y_m),norm(delx)]); display(serial_param_left.screw)
            i_start = i;
        else
            display('right='); display([norm(y_m),norm(delx)]); display(serial_param_right.screw)
            i_start = i;
        end
        
        J_m=[]; y_m=[]; A_m=[]; B_m=[];        
    end
    J_m = [J_m; J];
    y_m = [y_m; y];
    A_m = [A_m, LogSE3(delg_actual)];
    B_m = [B_m, LogSE3(delb)];
end


serial_param_left.screw
serial_param_left_actual.screw
serial_param_right.screw
serial_param_right_actual.screw
        