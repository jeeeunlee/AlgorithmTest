function map = getMap(i_start, serial_param_left, serial_param_right, serial_param_left_actual, serial_param_right_actual, walking_data_support, q_data_left, q_data_right)
global step_size;
map.J_m=[]; map.y_m=[]; map.A_m=[]; map.B_m=[];
map.i_start = i_start;

i=i_start;
map.left = 0; map.right=0;
while (i<1650)
    i = i+step_size;
    if(walking_data_support(i-step_size)==1 && walking_data_support(i)==1) %:left support
        map.left =1; map.right=0;
        serial_state_prev.q = q_data_left(i-step_size,:);
        serial_state.q = q_data_left(i,:);
        J = getJRecursive(serial_param_left,serial_state_prev,serial_state);
        
        serial_state_prev = forward_kinematics_POE(serial_state_prev, serial_param_left);
        serial_state = forward_kinematics_POE(serial_state, serial_param_left);
        delg = InverseSE3(serial_state_prev.T(:,:,end)) * serial_state.T(:,:,end);
        delb = InverseSE3(serial_state_prev.T(:,:,end-1)) * serial_state.T(:,:,end-1);
        
        serial_state_prev = forward_kinematics_POE(serial_state_prev, serial_param_left_actual);
        serial_state = forward_kinematics_POE(serial_state, serial_param_left_actual);
        delg_actual = InverseSE3(serial_state_prev.T(:,:,end)) * serial_state.T(:,:,end);
        
        y = LogSE3(delg_actual*InverseSE3(delg));
        
    elseif(walking_data_support(i-step_size)==2 && walking_data_support(i)==2) %:right support
        map.left =0; map.right=1;
        serial_state_prev.q = q_data_right(i-step_size,:);
        serial_state.q = q_data_right(i,:);
        J = getJRecursive(serial_param_right,serial_state_prev,serial_state);
        
        serial_state_prev = forward_kinematics_POE(serial_state_prev, serial_param_right);
        serial_state = forward_kinematics_POE(serial_state, serial_param_right);
        delg = InverseSE3(serial_state_prev.T(:,:,end)) * serial_state.T(:,:,end);
        delb = InverseSE3(serial_state_prev.T(:,:,end-1)) * serial_state.T(:,:,end-1);
        
        serial_state_prev = forward_kinematics_POE(serial_state_prev, serial_param_right_actual);
        serial_state = forward_kinematics_POE(serial_state, serial_param_right_actual);
        delg_actual = InverseSE3(serial_state_prev.T(:,:,end)) * serial_state.T(:,:,end);
        
        y = LogSE3(delg_actual*InverseSE3(delg));
    else
        map.i_end = i;
        break;
    end    
    
    map.J_m = [map.J_m; J];
    map.y_m = [map.y_m; y];
    map.A_m = [map.A_m, LogSE3(delg_actual)];
    map.B_m = [map.B_m, LogSE3(delb)];
    
end
map.i_end = i;
% map.delx = (transpose(map.J_m)*map.J_m)\transpose(map.J_m)*map.y_m ;

end