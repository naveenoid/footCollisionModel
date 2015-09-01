function animateLinkMotion( t,y,model,phi,timeScale )
%ANIMATELINKMOTION Animation of the motion of a single link (with a hard
%constraint surface)
%  Arguments
%  t - time stamps
%  y - state of the falling foot system
% phi - orientation of the inclined plane
% figNum - number of the figure in which plotting will take place
% timeScale - scaling of the time to speed up or slow the animation (1 is
% realtime).
    l2 = model.foot.length;
    w2 = model.foot.height;
    
    l1 = model.leg.length;
    m0 = model.upperbody.mass;
    
    figure();

    %% extracting points
    theta = y(:,8);
    
    w_1 = zeros(length(t),3);
    w_2 = zeros(length(t),3);
    w_0 = zeros(length(t),3);
    w_a = zeros(length(t),3);
    w_b = zeros(length(t),3);
    w_c = zeros(length(t),3);
    
    for tempT = 1:length(t)
       %x2
       w_2(tempT,:) =  y(tempT,1:3);
       
       %x1
       w_position_f2 = iDynTree.Position(y(tempT,1), y(tempT,2), y(tempT,3));
       w_R_f2 = iDynTree.Rotation();
       w_R_f2.fromMatlab(rotationFromQuaternion(y(tempT,4:7)));
        %transformations
       w_X_f2 = iDynTree.Transform(w_R_f2, w_position_f2);
        %model.foot.joint_X_frame < == > joint = f1, frame = f2
       f2_X_f1 = iDynTree.Transform(iDynTree.Rotation.RotX(theta(tempT)), model.foot.joint_X_frame);
       w_X_f1 = w_X_f2 * f2_X_f1; % for completeness sake but we do not integrate twist of link 1
       w_position_1 = w_X_f1.getPosition();
       w_1(tempT,:) = w_position_1.toMatlab()';
       
       %x0
       f1_position_f0 = iDynTree.Position(0,0, l1);
       w_position_f0 = w_X_f1*f1_position_f0;
       w_0(tempT,:) = w_position_f0.toMatlab()';
       
       %a
       f2_position_a = iDynTree.Position(0, l2, 0);
       f2_R_a = iDynTree.Rotation();
       f2_X_a = iDynTree.Transform(f2_R_a, f2_position_a);
       w_position_a = w_X_f2*f2_position_a;
       w_X_a = w_X_f2 * f2_X_a;
       w_a(tempT,:) = w_position_a.toMatlab()';
       
       %b
       a_position_b = iDynTree.Position(0,0,w2);
       w_position_b = w_X_a*a_position_b;
       w_b(tempT,:) = w_position_b.toMatlab();
       %c
       f2_position_c = iDynTree.Position(0,0,w2);
       w_position_c = w_X_f2*f2_position_c;
       w_c(tempT,:) = w_position_c.toMatlab()';
    end
    
    P1 = w_2;
    P2 = w_c;
    P3 = w_b;
    P4 = w_a;
    
    %% axis
    axis([-2*w2 2*l2*cos(phi)+2*w2 -(l1+l2*sin(phi)) l1+l2]);
    axis equal;
    
    %% drawing the plane
    xl = xlim;
    left_line = [xl(1), xl(1) * sin(-phi)];
    right_line = [xl(2), xl(2) * sin(-phi)];
    line([left_line(1),right_line(1)],[left_line(2),right_line(2)],'Color','k','LineWidth',4.0); hold on;
    
%     T1 = [0 0];
%     T2 = 2*l2*[cos(phi) -sin(phi)];
%     T3 = 2*l2*[0,-sin(phi)];
%     line([T1(1),T2(1)],[T1(2),T2(2)],'Color','k','LineWidth',4.0); hold on;
%     line([T2(1),T3(1)],[T2(2),T3(2)],'Color','k','LineWidth',4.0); hold on;
%     line([T3(1),T1(1)],[T3(2),T1(2)],'Color','k','LineWidth',4.0); hold on;
    % expected surface angle
    line([-2*w2 2*l2*cos(phi)+2*w2],[0 0],'Color','r','LineStyle','--');

    h2 = text(1.5*l2,1.5*l2,sprintf('t = %2.2f sec',0));
    
    %% plotting the initial position and the box corresponding to the foot link
    Px = [P1(1,2) P2(1,2) P3(1,2) P4(1,2) P1(1,2)];
    Py = [P1(1,3) P2(1,3) P3(1,3) P4(1,3) P1(1,3) ];
    h = plot([P1(1,2) P2(1,2) P3(1,2) P4(1,2) P1(1,2)],[P1(1,3) P2(1,3) P3(1,3) P4(1,3) P1(1,3) ]  );
    hlink = plot([w_1(1,2) w_0(1,2)],[w_1(1,3) w_0(1,3)],'lineWidth',3.0);
    hmass = plot(w_0(1,2),w_0(1,3),'ro','lineWidth',4,'MarkerSize',10);
    %h = plot([0 0 2*l 2*l 0],[l*sin(phi) (l*sin(phi)+2*w) (l*sin(phi)+2*w) l*sin(phi) l*sin(phi)],'Color','k','LineWidth',2); 

    %% axis
    axis([-2*w2 2*l2*cos(phi)+2*w2 -(0.5*l1+l2) l1+l2]);
   axis equal;

    h2 = text(1.5*l2,1.5*l2,sprintf('t = %2.2f sec',0));
    
    title('Foot Impact dynamics on inclined surfaces - animation');
    %% looping through time plotting the behaviour
    for i =2:1:length(t)
        set(h,'xdata',[P1(i,2) P2(i,2) P3(i,2) P4(i,2) P1(i,2)],'ydata',[P1(i,3) P2(i,3) P3(i,3) P4(i,3) P1(i,3)]);
        set(hlink,'xdata',[w_1(i,2) w_0(i,2)],'ydata',[w_1(i,3) w_0(i,3)],'lineWidth',3.0);
        set(hmass,'xdata',w_0(i,2),'ydata',w_0(i,3),'lineWidth',4,'MarkerSize',10);
        delete(h2);
        h2 = text(1.5*l2,1.5*l2,sprintf('t = %2.2f sec',t(i)));
      
        refreshdata
        drawnow
        pause(timeScale*(t(i) - t(i-1)));        
    end
    delete(h2);
end

