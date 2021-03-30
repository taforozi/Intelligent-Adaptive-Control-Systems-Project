%% Intelligent & Adaptive Automatic Control Systems
% Aforozi Thomais
% AEM: 9291

%% Clear Workspace
clc;
clear; 
close all;

%% Initialization
% real (unknown) parameters
global theta;
theta = [-0.018 0.015 -0.062 0.009 0.021 0.75];

%% A) Solve equations - Solution trajectory - plot
tspan = 0:0.1:600;
initialSystem = [deg2rad(25) deg2rad(2);...
                    0 0;
                    0.005 0.005;]; % in rad

for i = 1:3
[t, sys] = ode45('openloop', tspan, initialSystem(i,:));

phi = sys(:,1);
p = sys(:,2);

figure;
plot(t,phi,t,p,'LineWidth', 1.2);
grid on;
title(['Solution trajectory using the initial point $(\phi,p) = ($', num2str(initialSystem(i,1)), ...
       ',', num2str(initialSystem(i,2)),')'],'Interpreter','Latex');
xlabel('$$time [sec]$$','Interpreter','Latex');
ylabel('$$\phi(t),p(t)$$','Interpreter','Latex');
leg1 = legend('$\phi(t)$','$p(t)$');
set(leg1,'Interpreter','latex');
end

% figure;
% quiver(phi,p,p,theta1*phi + theta2*p + (theta3*abs(phi) + theta4*abs(p)).*p+ theta5*(phi).^3);

%% B) Limit Cycle & Phase Portrait
% Observation of the system behavior & its stability  

% inital points inside (1,2,3,4) - on (5) - outside (6) the cycle boundary
tspan = 0:0.1:600;
initialSystem = [-0.05 0.02;...
                 deg2rad(5) deg2rad(3);...
                 deg2rad(-20) deg2rad(-1); ...
                 deg2rad(25) deg2rad(-2); 
                 0.6 0; 
                 deg2rad(40) deg2rad(10)]; 
% In the last case (initial point outside the cycle boundary) 
% a warning will appear that is caused by the equation integration       

figure;              
for i = 1:length(initialSystem)
 
    [~, sys] = ode45('openloop', tspan, initialSystem(i,:));

    phi = sys(:,1);
    p = sys(:,2);

    if(i < 5) % inside
        subplot(2,2,i);
        plot(phi,p,'LineWidth', 1.2);
        grid on;
        title(['Trajectory for initial point ($\phi$,p)=(', ...
            num2str(initialSystem(i,1)),',', num2str(initialSystem(i,2)),')'],'Interpreter','Latex');
        xlabel('$\phi [rad]$','Interpreter','Latex','fontsize',12);
        ylabel('$p [rad/sec]$','Interpreter','Latex','fontsize',12);

    elseif(i < 7) % on, outside
        figure;
        plot(phi,p,'LineWidth', 1.2);
        grid on;
        title(['Trajectory for initial point ($\phi$,p)=(', ...
            num2str(initialSystem(i,1)),',', num2str(initialSystem(i,2)),')'],'Interpreter','Latex');
        xlabel('$\phi [rad]$','Interpreter','Latex','fontsize',12);
        ylabel('$p [rad/sec]$','Interpreter','Latex','fontsize',12);       
    end
end

%% E) MRAC applicability
% Choose flag = 1 for the first form of the reference input r(t)
% and flag = 2 for the second one
flag = 1;

global p12;
global p22;
global gamma_k;
global gamma_l;
global gamma_theta;

Aref = [0 1; -1 -1.4];
Q = eye(2);  
P = lyap(Aref',Q);
p12 = P(1,2); p22 = P(2,2);

gamma_k = 95; gamma_l = 85; gamma_theta = 54;

% simulation time
tspan = 0:0.001:140;
% initial conditions
initSystem = zeros(1,10); 
initSystem(1) = deg2rad(12);
[~, solved] = ode23(@(t,x) solvedsys(t,x,flag), tspan, initSystem);

Phii = solved(:,1);
Pii = solved(:,2);
phi_ref = solved(:,3);
p_ref = solved(:,4);
k1 = solved(:,5);
k2 = solved(:,6);
l = solved(:,7);
th_3 = solved(:,8);
th_4 = solved(:,9);
th_5 = solved(:,10);

% Estimated control gains
K_est = [k1 k2]; L_est = l;
Theta_est = [th_3 th_4 th_5];

% Control input u
r = r_ref(tspan,flag);
u = zeros(length(tspan),1);
for i=1:length(tspan)
 u(i) = - k1(i)*Phii(i) - k2(i)*Pii(i) + l(i)*r(i) - th_3(i)*abs(Phii(i))*Pii(i) ...
     - th_4(i)*abs(Pii(i))*Pii(i) - th_5(i)*(Phii(i))^3; 
end

% Tracking error
e_phi = Phii - phi_ref;
e_p = Pii - p_ref;
e = [e_phi; e_p];


%% Plot
% ö,öref,eö
figure;
subplot(2,1,1)
plot(tspan,Phii,'LineWidth', 1.8,'LineStyle','-.', 'Color', [0.8500 0.3250 0.0980]);
hold on;
plot(tspan,phi_ref,'LineWidth', 1.3, 'Color', [0 0.4470 0.7410]);
hold on;
plot(tspan,r_ref(tspan,flag),'LineWidth', 1.5,'Color', [0.9290 0.6940 0.1250]);
grid on;
title('Angle $\phi(t)$ and Reference input r(t)','Interpreter','Latex', 'fontsize', 12);
xlabel('time [sec]','Interpreter','Latex', 'fontsize', 12); 
ylabel('$\phi(t),\phi_{ref}(t), r_{ref}(t)$','Interpreter','Latex', 'fontsize', 10); 
legref = legend('$\phi_{real}$','$\phi_{ref}$','$r_{ref}$');
set(legref,'Interpreter','latex', 'fontsize', 12);
subplot(2,1,2)
plot(tspan,abs(e_phi),'LineWidth', 1.5);
grid on;
title('Absolute Error$_{\phi}$','Interpreter','Latex', 'fontsize', 12);
xlabel('time [sec]','Interpreter','Latex', 'fontsize', 12); 
ylabel('$e_{\phi}$','Interpreter','Latex', 'fontsize', 12); 


% p,pref,ep
figure;
subplot(2,1,1)
plot(tspan,Pii,'LineWidth', 1.8,'LineStyle','-.','Color',[0.8500 0.3250 0.0980]);
hold on;
plot(tspan,p_ref,'LineWidth', 1, 'Color', [0 0.4470 0.7410]);
grid on;
title('Angular velocity p(t)','Interpreter','Latex', 'fontsize', 12);
xlabel('time [sec]','Interpreter','Latex', 'fontsize', 12);
ylabel('$p(t),p_{ref}(t)$','Interpreter','Latex', 'fontsize', 12);
legrep = legend('$p_{real}$','$p_{ref}$');
set(legrep,'Interpreter','latex', 'fontsize', 12);
subplot(2,1,2)
plot(tspan,abs(e_p),'LineWidth', 1.5);
grid on;
title('Absolute Error$_{p}$','Interpreter','Latex', 'fontsize', 12);
xlabel('time [sec]','Interpreter','Latex', 'fontsize', 12);
ylabel('$e_{p}$','Interpreter','Latex', 'fontsize', 12);

% u: control input
figure;
plot(tspan,u,'LineWidth', 1.2);
grid on;
title('Control Input u(t)','Interpreter','Latex');
xlabel('time [sec]','Interpreter','Latex');
ylabel('$u(t)$','Interpreter','Latex');

% control gains
% \hat{k1} vs k1* and \hat{k2} vs k2*
realK = zeros(length(tspan),2);
realK(:,1) = 1.30933;
realK(:,2) = 1.8866;
figure;
subplot(2,1,1)
plot(tspan,realK(:,1),'LineWidth', 1.5);
hold on;
plot(tspan,k1,'LineStyle','-.','LineWidth', 1.5);
grid on;
title('Control gain $\hat{k_{1}}(t)$','Interpreter','Latex', 'fontsize', 12);
xlabel('time [sec]','Interpreter','Latex', 'fontsize', 12);
ylabel('$k^{*}_1, \hat{k}_1(t)$','Interpreter','Latex', 'fontsize', 12);
legk = legend('$k^{*}_{1}$','$\hat{k_{1}}(t)$');
set(legk,'Interpreter','latex', 'fontsize', 12);
subplot(2,1,2)
plot(tspan,realK(:,2),'LineWidth', 1.5);
hold on;
plot(tspan,k2,'LineStyle','-.','LineWidth', 1.5);
grid on;
title('Control gain $\hat{k_{2}}(t)$','Interpreter','Latex', 'fontsize', 12);
xlabel('time [sec]','Interpreter','Latex', 'fontsize', 12);
ylabel('$k^{*}_2, \hat{k}_2(t)$','Interpreter','Latex', 'fontsize', 12);
legk = legend('$k^{*}_{2}$','$\hat{k_{2}}(t)$');
set(legk,'Interpreter','latex', 'fontsize', 12);

% \hat{L} vs L* 
realL = zeros(length(tspan),1);
realL(:,1) = 1.33333;
figure;
plot(tspan,realL(:,1),'LineWidth', 1.5);
hold on;
plot(tspan,l,'LineStyle','-.','LineWidth', 1.5);
grid on;
title('Control gain $\hat{L}(t)$','Interpreter','Latex','fontsize', 12);
xlabel('time [sec]','Interpreter','Latex','fontsize', 12);
ylabel('$L, \hat{L}(t)$','Interpreter','Latex','fontsize', 12);
legk = legend('$L^{*}$','$\hat{L}(t)$');
set(legk,'Interpreter','latex');

% \hat{theta_3} vs theta_3*, \hat{theta_4} vs theta_4* 
% and \hat{theta_5} vs theta_5*
Theta_est = Theta_est*theta(6);
th_vec = zeros(length(tspan),3);
th_vec(:,1) = theta(3);
th_vec(:,2) = theta(4);
th_vec(:,3) = theta(5);
figure;
subplot(3,1,1)
plot(tspan,th_vec(:,1),'LineWidth', 1.5);
hold on;
plot(tspan,Theta_est(:,1),'LineStyle','-.','LineWidth', 1.5);
grid on;
title('Control gain $\hat{\theta}_3(t)$','Interpreter','Latex','fontsize', 12);
xlabel('time [sec]','Interpreter','Latex','fontsize', 12);
ylabel('$\theta_3,\hat{\theta}_3(t)$','Interpreter','Latex','fontsize', 12);
legt3 = legend('$\theta_3$','$\hat{\theta_3}(t)$');
set(legt3,'Interpreter','latex','fontsize', 12);
subplot(3,1,2)
plot(tspan,th_vec(:,2),'LineWidth', 1.5);
hold on;
plot(tspan,Theta_est(:,2),'LineStyle','-.','LineWidth', 1.5);
grid on;
title('Control gain $\hat{\theta}_4(t)$','Interpreter','Latex','fontsize', 12);
xlabel('time [sec]','Interpreter','Latex','fontsize', 12);
ylabel('$\theta_4,\hat{\theta}_4(t)$','Interpreter','Latex','fontsize', 12);
legt4 = legend('$\theta_4$','$\hat{\theta_4}(t)$');
set(legt4,'Interpreter','latex','fontsize', 12);
subplot(3,1,3)
plot(tspan,th_vec(:,3),'LineWidth', 1.5);
hold on;
plot(tspan,Theta_est(:,3),'LineStyle','-.','LineWidth', 1.5);
grid on;
title('Control gain $\hat{\theta}_5(t)$','Interpreter','Latex','fontsize', 12);
xlabel('time [sec]','Interpreter','Latex','fontsize', 12);
ylabel('$\theta_5,\hat{\theta}_5(t)$','Interpreter','Latex','fontsize', 12);
legt5 = legend('$\theta_5$','$\hat{\theta_5}(t)$');
set(legt5,'Interpreter','latex','fontsize', 12);

% print final errors
fprintf('Errors between estimated and real values: \n');
fprintf('k1: %f \n', (k1(end)-realK(end,1)));
fprintf('k2: %f\n', (k2(end)-realK(end,2)));
fprintf('l: %f\n', (l(end)-realL(end)));
fprintf('theta3: %f\n', (Theta_est(end,1)-theta(3)));
fprintf('theta4: %f\n', (Theta_est(end,2)-theta(4)));
fprintf('theta5: %f\n', (Theta_est(end,3)-theta(5)));



