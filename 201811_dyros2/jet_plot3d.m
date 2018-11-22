function jet_plot3d(robot_state, option)
figure(1)
cla reset;
p = zeros(3,7);
p(1:3,1:7)=robot_state.left_foot.T(1:3,4,:);
plot3([robot_state.base(1,4),p(1,:)],[robot_state.base(2,4),p(2,:)],[robot_state.base(3,4),p(3,:)],'o-');

hold on;
p = zeros(3,7);
p(1:3,1:7)=robot_state.right_foot.T(1:3,4,:);
plot3([robot_state.base(1,4),p(1,:)],[robot_state.base(2,4),p(2,:)],[robot_state.base(3,4),p(3,:)],'o-');

p = zeros(3,3);
p(1:3,1:3)=robot_state.waist.T(1:3,4,:);
plot3([robot_state.base(1,4),p(1,:)],[robot_state.base(2,4),p(2,:)],[robot_state.base(3,4),p(3,:)],'o-');

p = zeros(3,3);
p(1:3,1:3)=robot_state.head.T(1:3,4,:);
plot3([robot_state.waist.T(1,4,end),p(1,:)],[robot_state.waist.T(2,4,end),p(2,:)],[robot_state.waist.T(3,4,end),p(3,:)],'o-');

if(option.display.arm)
    p = zeros(3,8);
    p(1:3,1:8)=robot_state.left_hand.T(1:3,4,:);
    plot3([robot_state.waist.T(1,4,end),p(1,:)],[robot_state.waist.T(2,4,end),p(2,:)],[robot_state.waist.T(3,4,end),p(3,:)],'o-');

    p = zeros(3,8);
    p(1:3,1:8)=robot_state.right_hand.T(1:3,4,:);
    plot3([robot_state.waist.T(1,4,end),p(1,:)],[robot_state.waist.T(2,4,end),p(2,:)],[robot_state.waist.T(3,4,end),p(3,:)],'o-');
end
axis equal;grid on; xlabel('x');ylabel('y');zlabel('z');xlim([0-0.3,2]);
plot3(0,0,0,'X-')

plot3(robot_state.fixed_frame(1,4),robot_state.fixed_frame(2,4),robot_state.fixed_frame(3,4),'X-')

if(option.initialize)
    legend('l-foot','r-foot','waist','head','l-hand','r-hand');
end


end