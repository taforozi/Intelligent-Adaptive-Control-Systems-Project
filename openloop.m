% Open loop dynamics
function dx = openloop(t,x)

% define variables
phi = x(1);
p = x(2);

% init exercise constants
global theta;
% global theta6;

dx = zeros(size(x));

dx(1) = p;
dx(2) = theta(1) * phi + theta(2) * p + (theta(3) * abs(phi) + theta(4) * abs(p)) * p ...
        + theta(5) * (phi)^3;
    
end