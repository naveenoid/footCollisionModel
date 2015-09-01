function plotAndDebug(wholeSolution,model,environment, animate)
%PLOTANDDEBUG Summary of this function goes here
%   Detailed explanation goes here

figure();
rot = zeros(length(wholeSolution.t), 3);
rcPosition = zeros(length(wholeSolution.t), 3); %right corner position
f2_w_v_lc = zeros(length(wholeSolution.t), 6); %left corner twist, world orientation
f2_w_v_rc = zeros(length(wholeSolution.t), 6); %right corner twist, world orientation
for i = 1: length(wholeSolution.t)
    quat = wholeSolution.y(4:7, i);
    [roll,pitch,yaw] = rpyFromRotation(rotationFromQuaternion(quat'));
    rot(i, :) = [roll,pitch,yaw];
    
    w_R_f2 = iDynTree.Rotation();
    w_R_f2.fromMatlab(rotationFromQuaternion(quat));
    f2_w_X_f2 = iDynTree.Transform(w_R_f2, iDynTree.Position());
    w_X_f2 = iDynTree.Transform(w_R_f2, iDynTree.Position(wholeSolution.y(1, i),wholeSolution.y(2, i),wholeSolution.y(3, i)));
    f2_X_b = iDynTree.Transform(iDynTree.Rotation.Identity,...
        iDynTree.Position(0, model.foot.length, 0));
    
    b_w_X_f2_w = f2_w_X_f2 * f2_X_b;
    w_X_b = w_X_f2 * f2_X_b;
    
    f2_w_v_lc(i, :) = f2_w_X_f2.inverse().asAdjointTransform.toMatlab * wholeSolution.y(9:14, i);
    f2_w_v_rc(i, :) = f2_X_b.inverse().asAdjointTransform.toMatlab * f2_w_v_lc(i, :)';
    rcPosition(i, :) = w_X_b.getPosition().toMatlab();
end
plot(wholeSolution.t, rot(:,1)');
hold on;
plot(wholeSolution.t, wholeSolution.y(1:3, :));
legend('rotx','x','y','z');
yl = ylim;
for i = 1:length(wholeSolution.event)
    line([wholeSolution.event(i), wholeSolution.event(i)], yl, 'LineStyle', ':', 'Color', 'k'); 
end

figure()
hold on;
title('Velocity of left corner w.r.t. base')
plot(wholeSolution.t, f2_w_v_lc(:,1:3));
legend('x','y','z');
yl = ylim;
for i = 1:length(wholeSolution.event)
    line([wholeSolution.event(i), wholeSolution.event(i)], yl, 'LineStyle', ':', 'Color', 'k'); 
end

figure()
hold on;
title('Velocity of right corner w.r.t. base')
plot(wholeSolution.t, f2_w_v_rc(:,1:3));
legend('x','y','z');
yl = ylim;
for i = 1:length(wholeSolution.event)
    line([wholeSolution.event(i), wholeSolution.event(i)], yl, 'LineStyle', ':', 'Color', 'k'); 
end

figure();
plot(wholeSolution.t, wholeSolution.y(8, :));
legend('q');
yl = ylim;
for i = 1:length(wholeSolution.event)
    line([wholeSolution.event(i), wholeSolution.event(i)], yl, 'LineStyle', ':', 'Color', 'k'); 
end

figure();
plot(wholeSolution.t, rcPosition);
title('Position of right corner');
legend('x','y','z');
yl = ylim;
for i = 1:length(wholeSolution.event)
    line([wholeSolution.event(i), wholeSolution.event(i)], yl, 'LineStyle', ':', 'Color', 'k'); 
end


%%Animation
if nargin > 3
    if (animate)
        animateLinkMotion(wholeSolution.t,wholeSolution.y',model,environment.plane_inclination,10);
    end
end

end

