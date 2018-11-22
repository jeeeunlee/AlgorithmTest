function screw_new = update_screw(screw, del)
global alpha
    w = screw(1:3) + alpha*del(1:3);
    v = screw(4:6) + alpha*del(4:6);

% screw_new = LogSE3(LargeSE3(screw)*LargeSE3(alpha*del));
% w = screw_new(1:3);
% v = screw_new(4:6);

    % constraint : ||w+dw||=1, (w+dw)'(v+dv)=0
    w = w/norm(w);
    v = v - (w'*v) * w; 
    
    screw_new = [w;v];        
end