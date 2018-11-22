function [robot_param, robotClass] = get_robot_param_urdf(file_name)

robotClass = importrobot(file_name);
robot_param = get_robot_param_urdfClass(robotClass);

end
