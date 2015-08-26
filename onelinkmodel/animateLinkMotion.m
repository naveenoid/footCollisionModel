function [ output_args ] = animateLinkMotion( t,y,params,phi,figNum,speed )
%ANIMATELINKMOTION Animation of the motion of a single link (with a hard
%constraint surface)
%   
    l = params.l;
    w = params.w;
    fExt = params.fExt;
    pfExt = params.pfExt;

    figure(figNum);

    %% extracting points
    theta = y(:,1);
    P1 = zeros(length(theta),2);
    P2 = 2*w*[-cos(pi/2 - theta)  sin(pi/2-theta)];
    P4 = 2*l*[cos(theta) sin(theta)];
    P3 = P2 + P4;
    
    %% drawing the spring
    turns=2; %The number of turns the spiral will have


    
    %% drawing the plane
    T1 = [0 0];
    T2 = 2*l*[cos(phi) -sin(phi)];
    T3 = 2*l*[0,-sin(phi)];
    line([T1(1),T2(1)],[T1(2),T2(2)],'Color','k','LineWidth',4.0); hold on;
    line([T2(1),T3(1)],[T2(2),T3(2)],'Color','k','LineWidth',4.0); hold on;
    line([T3(1),T1(1)],[T3(2),T1(2)],'Color','k','LineWidth',4.0); hold on;
    % expected surface angle
    line([-2*w 2*l*cos(phi)+2*w],[0 0],'Color','r','LineStyle','--');
    
    %% drawing the external force
    pF1 = [P2(:,1)+pfExt*cos(theta) , P2(:,2)+pfExt*sin(theta)];
    pF2 = [pF1(:,1), pF1(:,2)+fExt*0.0005];
    
    hf = line([pF1(1,1),pF2(1,1)],[pF1(1,2),pF2(1,2)],'Color','g','LineWidth',2.0);
    hft = text(pF2(1,1),pF2(1,2)+0.05,sprintf('F_ext = %2.2fN',fExt));
    %% plotting the initial position
    Px = [P1(1,1) P2(1,1) P3(1,1) P4(1,1) P1(1,1)];
    Py = [P1(1,2) P2(1,2) P3(1,2) P4(1,2) P1(1,2) ];
    h = plot([P1(1,1) P2(1,1) P3(1,1) P4(1,1) P1(1,1)],[P1(1,2) P2(1,2) P3(1,2) P4(1,2) P1(1,2) ]  );
    %h = plot([0 0 2*l 2*l 0],[l*sin(phi) (l*sin(phi)+2*w) (l*sin(phi)+2*w) l*sin(phi) l*sin(phi)],'Color','k','LineWidth',2); 
    
    
    %% axis
    axis([-2*w 2*l*cos(phi)+2*w -2*l*sin(phi) 2*l]);
    axis square;

    h2 = text(1.5*l,1.5*l,sprintf('t = %2.2f sec',0));
 
    for i =2:2:length(t)
%        rotate(h, [0 0 1], y(i,1) - y(i-1),[0 l*sin(phi) 0]);
        set(h,'xdata',[P1(i,1) P2(i,1) P3(i,1) P4(i,1) P1(i,1)],'ydata',[P1(i,2) P2(i,2) P3(i,2) P4(i,2) P1(i,2)]);
        delete(h2);
        h2 = text(1.5*l,1.5*l,sprintf('t = %2.2f sec',t(i)));
        delete(hf);
        delete(hft);
        hf = line([pF1(i,1),pF2(i,1)],[pF1(i,2),pF2(i,2)],'Color','g','LineWidth',2.0);
        hft = text(pF2(i,1),pF2(i,2)+0.05,sprintf('F_{ext} = %2.2fN',fExt));
   
        refreshdata
        drawnow
        pause(speed*(t(i) - t(i-1)));        
    end

end

