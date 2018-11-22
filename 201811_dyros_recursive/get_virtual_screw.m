function [v_serial_param_left,v_serial_param_right] = get_virtual_screw(serial_param_left,serial_param_right,foot_dof)

v_serial_param_left = serial_param_left;
v_serial_param_right = serial_param_right;
dof = serial_param_left.dof;

M_accum_left = eye(4);
M_accum_right = eye(4);
for i = 1 : foot_dof
    v_serial_param_left.M(:,:,i) = serial_param_left.M(:,:,i) * errorSE3(0.01); 
    M_accum_left = M_accum_left * v_serial_param_left.M(:,:,i);
    v_serial_param_left.screw(:,i) = Adj(M_accum_left, serial_param_left.S(:,:,i));
    
    v_serial_param_right.M(:,:,i) = serial_param_right.M(:,:,i) * errorSE3(0.01);
    M_accum_right = M_accum_right * v_serial_param_right.M(:,:,i);    
    v_serial_param_right.screw(:,i) = Adj(M_accum_right, serial_param_right.S(:,:,i));
end

for i = foot_dof+1 : dof
    v_serial_param_left.M(:,:,i) = serial_param_left.M(:,:,i) * errorSE3(0.01); 
    v_serial_param_right.M(:,:,i) = v_serial_param_left.M(:,:,i);
    
    M_accum_left = M_accum_left * v_serial_param_left.M(:,:,i);
    v_serial_param_left.screw(:,i) = Adj(M_accum_left, serial_param_left.S(:,:,i));
    
    M_accum_right = M_accum_right * v_serial_param_right.M(:,:,i);
    v_serial_param_right.screw(:,i) = Adj(M_accum_right, serial_param_right.S(:,:,i));    
end

v_serial_param_left.tcp = serial_param_left.tcp * errorSE3(0.001);
v_serial_param_right.tcp = v_serial_param_left.tcp;

v_serial_param_left.screw(:,dof+1) = LogSE3(M_accum_left * v_serial_param_left.tcp);
v_serial_param_right.screw(:,dof+1) = LogSE3(M_accum_right * v_serial_param_right.tcp);


    
end