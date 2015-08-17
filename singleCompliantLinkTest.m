clear
close all

%% initial conditions
theta_ini = pi/4;
thetaDot_ini = 0;
x_ini = [theta_ini;thetaDot_ini];

phi = pi/16; % slope angle

%% parameters
l = 0.3; % length of link
w = 0.05; % width of link
m = 0.5; % mass of link
g = 9.8;%9.81;%9.81; % gravity magnitude
tau = 0; % applied torque apart from impedance
k = 5.5; % joint stiffness
theta0 = 0; % rest position of impedance control
b = 0.1; % joint damping
e = 0.4; % coefficient of restitution
fExt = 22.5*g; % Downward Body force exerted on top of link at a distance from corner
pfExt = 0.1*l; % application point of external force from top left corner
dExt = ((2*w).^2 + pfExt.^2).^0.5;
alpha = atan2(pfExt,2*w); % refer to diagram (angle of arc from top-left corner to force application point)
betaExt = @(theta)(180 - theta+alpha); % angle between forces

params.l = l; params.w = w;
params.m = m; params.g = g;
params.Iinv = 0.75/(m * (l.^2 + w.^2));
params.betaExt = betaExt;
params.fExt = fExt;
params.dExt = dExt;
params.pfExt = pfExt;

%% simulation settings
tmin = 0;
tmax = 10;

tstart = 0;
tend = 0;

figure(1);
yStore = cell(1);
tStore = cell(1);

%% integration system and options
func = @(t,x)singleCompliantLink_planar(t,x,tau,k,b,theta0,params);
options = odeset('Events',@(t,y)linkPlaneCollisionEvent(t,y,phi),'OutputFcn', @odeplot,...
                  'OutputSel',1,'Refine',4);

idxCnt = 0;
tLength = 0;

%% loop runs animation between bounces
while (tend ~=tmax)
   idxCnt = idxCnt+1;
  % fprintf('%d, ',idxCnt);
   
   %% integrate system
   [t,y] =  ode45(func, [tstart tmax], x_ini',options);
   tend = t(end);
   tstart = t(end);
   x_ini = y(end,1:2)';
   x_ini(2) = -e*x_ini(2);
   fprintf('Bounce %d @ time %2.2f sec, new state [ %2.3f,%2.3f]\n',idxCnt,t(end),x_ini(1),x_ini(2));
   plot(t,y(:,1)); hold on;
   
   yStore{idxCnt} = y;
   tStore{idxCnt} = t;
   
   tLength = tLength + length(t);
   
   if(x_ini(2) <= 1e-4)
      break;
      fprintf('Terminating simulation..\n');
   else
       fprintf('Continuing execution..\n');
   end
   
end    

xlabel('time (sec)');
ylabel('theta (rads)');


%% consilidating dataseries for animating
fprintf('\n-----------------\nStarting animation\n');

yAnimate = zeros(tLength, 2);
tAnimate = zeros(tLength,1);
idxSoFar = 1;
for i = 1:idxCnt
    tNowLen = length(tStore{i});
    tAnimate( idxSoFar : idxSoFar + tNowLen-1) = tStore{i};
    yAnimate( idxSoFar : idxSoFar + tNowLen-1,:) = yStore{i};
    idxSoFar = idxSoFar + tNowLen;
end

%% animation
animateLinkMotion(tAnimate,yAnimate,params,phi,2);
