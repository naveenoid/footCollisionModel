function [value,isterminal,direction] = linkPlaneCollisionEvent(t,y,phi)
% Locate the time when height passes through zero in a 
% decreasing direction and stop integration.
value = y(1) + phi;    % Detect theta + phi= 0
isterminal = 1;   % Stop the integration
direction = -1;   % Negative direction only
end