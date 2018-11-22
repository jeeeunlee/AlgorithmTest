function robot_param = get_robot_param(model_name)

Init_Constant;

robot_param = struct;


switch model_name
    case 'jet'
        robot_param.left_foot.dof = 6;
        robot_param.right_foot.dof = 6;
        robot_param.left_arm.dof = 7;
        robot_param.right_arm.dof = 7;
        robot_param.torso.dof = 2;
        robot_param.head.dof = 2;
        
        robot_param.dof = robot_param.left_foot.dof + robot_param.right_foot.dof ...
            + robot_param.left_arm.dof +robot_param.right_arm.dof + robot_param.torso.dof + robot_param.head.dof;
        dof = robot_param.dof;
        
        % Reduction ratio
        robot_param.transmission = zeros(1,dof); % reduction ratio
        
        % RPM
        robot_param.max_rpm = [3500,3500,3500,3500,3500,3500]; % rpm
        
        % LIMIT
        robot_param.max_limit = [170, 90, 155, 170, 130, 360];
        robot_param.min_limit = [-170, -130, -80, -170, -130, -360];
        
        
        % MAX ACC
        robot_param.joint_max_acc = [15,15,15,15,15,15]; % rad/s2
        
        % MAX VEL
        robot_param.joint_max_vel = 2*pi/SEC2MIN*robot_param.max_rpm./robot_param.transmission;
        
        % MAX TORQUE
        ratio = 0.85;
        robot_param.max_torque = [662, 612, 204, 96, 70, 56] * ratio;
        
        % MASS
        mass = [32.7, 8.2, 11.9, 4.0, 1.2, 7.36];
        
        % Initial Configuration
        robot_param.M = zeros(4,4,dof);
        
        % link 1
        robot_param.M(1:3,1:3,1) = [ 1 0 0 ; 0 -1 0 ; 0 0 -1];
        robot_param.M(1:3,4,1) = [ 0 0 0.0]';
        robot_param.M(4,4,1) = 1;
        
        % link 2
        robot_param.M(1:3,1:3,2) = [ 1 0 0 ; 0 0 1 ; 0 -1 0];
        robot_param.M(1:3,4,2) = [0.095, 0, -0.3685]';
        robot_param.M(4,4,2) = 1;
        
        % link 3
        robot_param.M(1:3,1:3,3) = [ 1 0 0 ; 0 1 0 ; 0 0 1];
        robot_param.M(1:3,4,3) = [0, 0.435, 0]';
        robot_param.M(4,4,3) = 1;
        
        % link 4
        robot_param.M(1:3,1:3,4) = [ 0 0 1 ; 0 1 0 ; -1 0 0];
        robot_param.M(1:3,4,4) = [0, 0.1, 0]';
        robot_param.M(4,4,4) = 1;
        
        % link 5
        robot_param.M(1:3,1:3,5) = [ 0 0 1 ; 1 0 0 ; 0 1 0];
        robot_param.M(1:3,4,5) = [0, 0.0, 0.075 + 0.353]';
        robot_param.M(4,4,5) = 1;
        
        % link 6
        robot_param.M(1:3,1:3,6) = [ 1 0 0 ; 0 0 -1 ; 0 1 0];
        robot_param.M(1:3,4,6) = [0, 0.08 + 0.016, 0]';
        robot_param.M(4,4,6) = 1;
        
        T_m = zeros(4,4,dof);
        T_accum = eye(4);
        for i = 1 : dof
            T_accum = T_accum * robot_param.M(:,:,i);
            T_m(:,:,i) = T_accum;
        end
        
        
        % Screw for each joint
        robot_param.S = zeros(6,1,dof);
        for i = 1 : dof
            robot_param.S(:,:,i) = [0,0,1,0,0,0]';
        end
        
        
        
        % Geleralized Inertia for each link
        robot_param.J = zeros(6,6,dof);
        
        
        % center of mass parameters
        J_com = zeros(6,6,dof);
        T_com = zeros(4,4,dof);
        I_com = zeros(6,dof);
        
        
        %  Ixx, Iyy, Izz, Iyz, Ixz, Ixy
        I_com(:,1) = [361505 356886 254737 21618 94123 1301]' * mm2m_sqr ;
        I_com(:,2) = [239628 245343 13846 3716 -4238 -223]' * mm2m_sqr;
        I_com(:,3) = [67535 56165 48121 10808 6016 -4419]' * mm2m_sqr;
        I_com(:,4) = [21234 101792 112006 23 -260 -12582]' * mm2m_sqr;
        I_com(:,5) = [1841 3926 4017 2.1 -18.6 34.1]' * mm2m_sqr;
        I_com(:,6) = [12638 25489 20764 2.0 -4037 -0.0]' * mm2m_sqr;
        
        P_com(:,1) = [38.9 9.7 267.9+14]' * mm2m;
        P_com(:,2) = [101.4 -93.79 547.8+14]' * mm2m;
        P_com(:,3) = [111.6 15.7 843.4+14]' * mm2m;
        P_com(:,4) = [391.3 -18.0 890.1+14]' * mm2m;
        P_com(:,5) = [573.7 -0.34 889.6+14]' * mm2m;
        P_com(:,6) = [95+75+353+80+16, 0, 190+178.5+435+100]' * mm2m + [81.03, 0, -101.36]' * mm2m;
    
        % P_com(:,6) = [702.9 0.0 792.5+14]' * mm2m;
        
        
        
        for i = 1 : dof
            J_com(1:3,1:3,i) = [ I_com(1,i) I_com(6,i) I_com(5,i);
                I_com(6,i) I_com(2,i) I_com(4,i);
                I_com(5,i) I_com(4,i) I_com(3,i)];
            J_com(4:6,4:6,i) = mass(i)*eye(3);
            T_com(1:3,4,i) = P_com(:,i);
            
            % TODO: Rotation matrix must be set.
            T_com(1:3,1:3,i) = eye(3);
            % T_com(1:3,1:3,i) = R_com(:,:,i);
            
            T_com_to_frame = InverseSE3(T_com(:,:,i)) * T_m(:,:,i);
            Ad_i = Ad_T(T_com_to_frame);
            
            robot_param.J(:,:,i) = Ad_i' * J_com(:,:,i) * Ad_i;
            
        end

        
end



% Base link Velocity
robot_param.V0 = zeros(dof,1);

% Base link Acceleration
robot_param.dV0 = [ zeros(3,1) ; -gravity ];


% External Force exerted on each link
robot_param.Fext = zeros(6,1,dof);

% External Force frame attached at each link
robot_param.ft = zeros(4,4,dof);

% TCP
robot_param.tcp = eye(4);






