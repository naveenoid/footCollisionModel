function [ dx ] = singleCompliantLink_planar( t,x,tau,k,b,theta0,param )
%SINGLECOMPLIANTLINK_PLANAR ODE of a single planar link as a rigid body
%with control compliance
% Arguments : 
% t - time
% x - state of the link expressed as [theta;dtheta]
% tau - torque as a function of time
% k - stiffness of impedance controller
% b - dampoing of impedance controller
% theta0 - Rest position of the impedance controller
% params - structure containing following parameters
% m - mass of the link
% l - half the lenght of the link
% w - half the width of the link
% g - gravity magnitude-theta_ini

    l = param.l;
    w = param.w;
    m = param.m;
    g = param.g;
    Iinv = param.Iinv;
    betaExt = param.betaExt;
    fExt = param.fExt;
    dExt = param.dExt;
    
    theta = x(1);
    thetaDot = x(2);
    
    tauExt = fExt * dExt * sin(betaExt(theta));
    thetaDDot = Iinv*( tauExt + tau-k*(theta-theta0) - b*(thetaDot) - m*g*l*cos(theta));

    dx = [thetaDot;thetaDDot];
end

