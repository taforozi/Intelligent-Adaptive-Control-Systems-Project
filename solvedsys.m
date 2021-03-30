% Closed loop dynamics
function dx = solvedsys(t,x,flag)

% define variables
phi = x(1);
p = x(2);
phi_ref = x(3);
p_ref = x(4);
k = [x(5), x(6)];
l = x(7);
xtheta = [x(8), x(9), x(10)];

% init constants
global theta;
% pij variables & control gains
global p12;
global p22;
global gamma_k;
global gamma_l;
global gamma_theta;

dx = zeros(size(x));

% Control input u
  u = -k * [phi; p] + l * r_ref(t,flag) - xtheta * [abs(phi)*p; abs(p)*p; (phi)^3];

% Real system
  dx(1) = p;
  dx(2) = theta(1) * phi + theta(2) * p + (theta(3) * abs(phi) + theta(4) * abs(p)) * p + theta(5) * (phi)^3 + theta(6) * u;
% Model Reference
  dx(3) = p_ref;
  dx(4) = - 1 * phi_ref - 1.4 * p_ref + r_ref(t,flag);
% Control gains K(t) = [k1 k2]
  dx(5) = gamma_k * (p12 * (phi - phi_ref) + p22 * (p - p_ref)) * phi;
  dx(6) = gamma_k * (p12 * (phi - phi_ref) + p22 * (p - p_ref)) * p;
% Control gain L(t)
  dx(7) = - gamma_l * (p12*(phi - phi_ref) + p22*(p - p_ref))*r_ref(t,flag);
% Control gains È(t) = 1/è6*[è3; è4; è5]  
  dx(8) = gamma_theta * (abs(phi) * p * (phi - phi_ref) * p12 + abs(phi) * p * (p - p_ref) * p22);
  dx(9) = gamma_theta * (abs(p) * p * (phi - phi_ref) * p12 + abs(p) * p * (p - p_ref) * p22);
  dx(10) = gamma_theta * ((phi^3) * (phi - phi_ref) * p12 + (phi^3) * (p - p_ref) * p22);

end