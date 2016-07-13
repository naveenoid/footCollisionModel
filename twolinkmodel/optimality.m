close all;
clear;
% 
% e4 = zeros(6,1); e4(4) = 1;
% e3 = zeros(6,1); e3(3) = 1;
% 
% y = sym('y', [2,1]);
% x = y * [0;0;0;1;0;0]';
% % x = sym('x', [2, 6]);
% beta = sym('beta', [2, 1]);
% 
% alpha = sym('alpha', [6, 1]);
% 
% fc = x' * beta + alpha;
% 
% pdes = sym('pdes');
% 
% J = 0.5 * (e4' * fc / (e3' * fc))^2;
% 
% dJdb = [diff(J, beta(1)), diff(J, beta(2))];
% 
% beta_star = solve(dJdb == 0, beta);
% 


Ihat = rand(6);
S = zeros(6,1); S(4) = 1;
t = rand(2,1);
x = -t * S';
alpha = rand(6,1);

e3 = zeros(6,1); e3(3) = 1;
e4 = zeros(6,1); e4(4) = 1;
pstar = rand(1);

y = -[Ihat(4,4) * t(1); Ihat(4,4) * t(2)];
z = -[Ihat(3,4) * t(1); Ihat(3,4) * t(2)];

% beta = rand(2,1); 
beta = - (1/((norm(y - pstar * z)^2)))*...
    (y - pstar * z)  * (alpha(4) - pstar* alpha(3));

betaCoeff = (y' - pstar * z');
(y' - pstar * z') * beta + (alpha(4) - pstar*alpha(3));

N = [1; -betaCoeff(1)/betaCoeff(2)];
N = N/norm(N);
betaCoeff * N;
% beta = beta + N * rand(1);
% (y' - pstar * z') * beta + (alpha(4) - pstar*alpha(3));

h = x * Ihat' * (e4 - pstar * e3);
beta = -pinv(h', 1e-5) * ((e4' - pstar * e3') * alpha);

N = [1; -h(1)/h(2)];
N = N / norm(N);

beta = beta + N * rand(1);

% beta = - (h / (norm(h)^2)) * (alpha(4) - pstar * alpha(3))

% (y' * beta + alpha(4))/(z' * beta + alpha(3)) - pstar

% beta = rand(2,1);
fc = Ihat * x' * beta + alpha;
cop = (e4' * fc) / (e3' * fc);
J = 0.5 * (cop - pstar)^2


