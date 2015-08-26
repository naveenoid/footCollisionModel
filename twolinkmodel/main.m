function main()

addpath(genpath('utilities'));

clear;
clc;
close all;

environment.plane_inclination = 10 / 180 * pi;

%%Model: two links - One dof
model = struct();
%note: numbers are random for now :D
%foot: box
model.foot.mass = 0.2; %kg
model.foot.length = 0.2; %m
model.foot.height = 0.05; %m
%foot reference frame: bottom-left angle
rotI = iDynTree.RotationalInertiaRaw();
rotI.zero();
rotI.setVal(1,1,1); rotI.setVal(2, 2,1);
rotI.setVal(0, 0, model.foot.mass/3 * (model.foot.length^2 + model.foot.height^2));
pos = iDynTree.Position(0, ...
    model.foot.length / 2, model.foot.height / 2);
model.foot.I = iDynTree.SpatialInertia(model.foot.mass, pos, rotI);
model.foot.joint_X_frame = iDynTree.Position(0, model.foot.length / 3, model.foot.height);

%leg: rod with uniform mass + an additional mass attached to one end.
%reference frame at the joint (one end of the rod)
model.leg.length = 0.4;
model.leg.mass = 10;
model.upperbody.mass = 20;
rotI.zero();
rotI.setVal(1,1,1); rotI.setVal(2, 2,1);
rotI.setVal(0, 0, model.leg.mass/3 * model.leg.length^2);
legI = iDynTree.SpatialInertia(model.leg.mass, iDynTree.Position(0, 0, model.leg.length / 2), rotI);
rotI.zero();
rotI.setVal(1,1,1); rotI.setVal(2, 2,1);
rotI.setVal(0,0, model.upperbody.mass * model.leg.length^2);
upperBodyI = iDynTree.SpatialInertia(model.upperbody.mass, iDynTree.Position(0, 0, model.leg.length), rotI);
model.leg.I = legI + upperBodyI;

%%Initial state
q = pi/2;
v2 = iDynTree.Twist();
v2.zero();
v2.setVal(1, 0);
v2.setVal(3, 0.2);
qdot = 0;

xpos_0 = [0; 
          0; 
          10;...
          quaternionFromEulerRotation(0.3, [1;0;0]);...
           q];
       
xdot_0 = [v2.toMatlab(); qdot];
x0 = [xpos_0; xdot_0];

fc = iDynTree.Wrench();
fc.zero();

tspan = [0, 10];

options = odeset('OutputFcn', @odeplot,...
                  'OutputSel',[1:3],'Refine',4,...
                  'RelTol', 1e-5, ...
                  'Events', @(t,y)collisionDetection(t,y,environment,model));

%first state: free flying state              
odesol =  ode45(@(t,x)odefunc(t, x, fc, model), ...
                tspan, x0', options);
   
if (odesol.x(end) == tspan(end))
    fprintf('No collision detected before t=%f\n',odesol.x(end));
else
    twistAtImpact = odesol.y(end,9:end-1);
    fprintf('Collision detected at time t=%f for event %d\n',odesol.x(end), odesol.ie);
    disp('Twist at impact is')
    disp(twistAtImpact')
    %I should start again to integrate but
    %I need to:
    % - v_0 = v^- (angular), 0 * v^- (linear) (only angular twist is
    % preserved)
    % - add position constraint of the contact point
    
    %reset initial state
    x0 = odesol.y(end,:);
    x0(9:11) = 0; %set linear twist to zero
    tspan(1) = odesol.x(end);
    
    options = odeset('OutputFcn', @odeplot,...
                  'OutputSel',[1:3],'Refine',4,...
                  'RelTol', 1e-5, ...
                  'Events', @(t,y)fullContactCondition(t,y,environment,model));

    %second state: establishing full contact
%     odesol =  ode45(@(t,x)odefunc(t, x, fc, model), ...
%                     tspan, x0, options);
%    
end

% figure();
% rot = zeros(length(t), 3);
% for i = 1: length(t)
%     quat = y(i, 4:7);
%     [roll,pitch,yaw] =  rpyFromRotation(rotationFromQuaternion(quat'));
%     rot(i, :) = [roll,pitch,yaw];
% end
% 
% plot(t, rot);
% plot(acc(1,:),acc(2,:)); %x linear
% hold on;
% plot(acc(1,:),acc(6,:)); %y angular
% plot(acc(1,:),acc(7,:)); %z angular

end

function dx = odefunc(t,x, fc, model)
% global acc Fs;
    damp = 3;
    
    xpos = x(1:7); %base position
%     q = x(8); %joint position
    xdot = x(9:9+5); %base velocity
    qdot = x(9+6); %joint velocity
    quatDot = quaternionDerivative(xpos(4:end), xdot(4:end), 1);
    
    u =0;% - damp * qdot;
    [a, F] = twolink_dynamic(t, x, u, fc, model);
    
    dx = [xdot(1:3);
          quatDot;
          qdot;
          a];
%       acc = [acc, [t;a]];
%       Fs = [Fs, [t; F]];
end

function [value,isterminal,direction] = collisionDetection(~,y, environment, model)
    xpos = y(1:7); %base position
    xposLin = xpos(1:3);
    xposRotation = rotationFromQuaternion(xpos(4:7));
    
    %check the lowest (z) point of the foot
    x1Pos = xposLin;
    x2Pos = xposLin + xposRotation * [0;model.foot.length; 0];
   
    value(1) = x1Pos(3) + x1Pos(2) * sin(environment.plane_inclination);
    value(2) = x2Pos(3) + x2Pos(2) * sin(environment.plane_inclination);
    
    isterminal = [1,1];
    direction = [-1, -1];    
end

function [value,isterminal,direction] = fullContactCondition(~,y, environment, model)
    xpos = y(1:7); %base position
    xposLin = xpos(1:3);
    xposRotation = rotationFromQuaternion(xpos(4:7));
    
    %check the lowest (z) point of the foot
    x1Pos = xposLin;
    x2Pos = xposLin + xposRotation * [0;model.foot.length; 0];
    
    distance = x1Pos(2);
    height = x1Pos(3);
    if (x2Pos(3) < x1Pos(3));
        distance = x2Pos(2);
        height = x2Pos(3);
    end
    
    value = height + distance * sin(environment.plane_inclination);
    
    isterminal = 1;
    direction = -1;    
end

