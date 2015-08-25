function main()

clear;
clc;

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
qdot = 0;

xpos_0 = [0; 
          0; 
          10;...
          quaternionFromEulerRotation(0, [1;0;0]);...
           q];
xdot_0 = [v2.toMatlab(); qdot];
x0 = [xpos_0; xdot_0];

fc = iDynTree.Wrench();
fc.zero();

tspan = [0, 10];

options = odeset('OutputFcn', @odeplot,...
                  'OutputSel',[1:3],'Refine',4,...
                  'RelTol', 1e-5);

global acc Fs;
acc = [];
Fs = [];
              
[t,y] =  ode45(@(t,x)odefunc(t, x, fc, model), ...
       tspan, x0', options);

%    figure();
% plot(acc(1,:),acc(2,:)); %x linear
% hold on;
% plot(acc(1,:),acc(6,:)); %y angular
% plot(acc(1,:),acc(7,:)); %z angular

end

function dx = odefunc(t,x, fc, model)
global acc Fs;
    [a, F] = twolink_dynamic(t, x, 0, fc, model);
    
    xpos = x(1:7); %base position
%     q = x(8); %joint position
    xdot = x(9:9+5); %base velocity
    qdot = x(9+6); %joint velocity
    quatDot = quaternionDerivative(xpos(4:end), xdot(4:end), 1);
    
    dx = [xdot(1:3);
          quatDot;
          qdot;
          a];
      acc = [acc, [t;a]];
      Fs = [Fs, [t; F]];
end

function status = myodeplot(t,y,flag)
if (status == 'init')
    %do init stuff
elseif (status == 'done')
    %do done stuff
else
    
end
status = 0;
end