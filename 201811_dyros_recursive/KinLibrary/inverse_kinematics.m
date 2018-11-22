function [robot_state, bsolved] = inverse_kinematics(target_T, robot_state, robot_param)

max_iter = 50;

invT = InverseSE3(target_T);

for i = 1 : max_iter

    robot_state = get_jacobian(robot_state, robot_param);
    dx = LogSE3(invT * robot_state.T(:,:,end));
    dq = pinv(robot_state.J) * dx;
    robot_state.q = robot_state.q - dq;
    if norm(dq) < 1e-5
        bsolved = true;
        return
    end
end
bsolved = false;
