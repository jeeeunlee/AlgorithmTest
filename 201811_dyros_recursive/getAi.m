function Ai = getAi(Si, qi)
% [del(exp([Si]qi))*exp(-[Si]qi)]v = Ai*del(Si) 3*

    wi = Si(1:3);
    vi = Si(4:6);

    w_norm = norm(wi);
    theta_i = w_norm*qi;
    skew_Si = [skew(wi), zeros(3); skew(vi), skew(wi) ];

    if w_norm > 1e-15
        Ai = qi*eye(6) + (4 - theta_i*sin(theta_i) - 4*cos(theta_i))/(2*w_norm^2) * skew_Si ...
            + (4*theta_i - 5*sin(theta_i) + theta_i*cos(theta_i))/(2*w_norm^3) * skew_Si^2 ...
            + (2 - theta_i*sin(theta_i) - 2*cos(theta_i))/(2*w_norm^4) * skew_Si^3 ...
            + (2*theta_i - 3*sin(theta_i) + theta_i*cos(theta_i))/(2*w_norm^5) * skew_Si^4;

    else
%         Ai = qi*eye(6);
        Ai = qi*eye(6) + qi/2*skew_Si;
    end
end