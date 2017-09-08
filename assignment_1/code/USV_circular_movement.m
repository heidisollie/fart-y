%% Task 2.3
close all;
clc;
% some constants
deg2rad = pi/180;   
rad2deg = 180/pi;

% current parameters
U_c = 0.6; % m/s
alpha_c = 10 * deg2rad;
beta_c = 45 * deg2rad;

% Euler anlges of vechicle
phi = 0 *deg2rad;
theta = 2.0 *deg2rad;
psi = 30 * deg2rad;

R_n_b = Rzyx(phi,theta,psi);            % Rotation matrix from ned to body


v_n_c = [U_c*cos(alpha_c)*cos(beta_c);
         U_c*sin(beta_c);
         U_c*sin(alpha_c)*cos(beta_c)]; % Current velocity in ned
     
v_b_c = inv(R_n_b)*v_n_c;               % current velocity in body             

%% Task 2.4   Simulation and plotting

% Without current
t_end = 419*3;
h = 0.1;
N = t_end/h;
U = 1.5;
w = 0.859 * deg2rad ;                   % turning rate  of vechicle based on circular motion

v_n_b = zeros(N, 3);                    % velocities of body  realtive to ned in body coordinates                     
p_n_b = zeros(N, 3);                    % position of body realative to ned in ned coordinates
v_n_b_relative = zeros(N, 3);                  % velocities of body with current  realtive to ned in body coordinates
p_n_b_c = zeros(N, 3);                  % position of body with current realative to ned in ned coordinates
v_n_b_n =zeros(N, 3); 
p_n_current = zeros(N,3);

crab_angle = zeros(N,1);
sideslip_angle = zeros(N,1);


%[J,R_nb,T] = eulerang(phi,theta,psi);

for i = 0:N-1;
    t = i*h;
    
    %without current
    v_b_b = [U*cos(w*t); U*sin(w*t); 0];
    v_n_b(i+1,:) = (R_n_b * v_b_b)';
    
    % relative velocity
    v_b_b_relative = [U*cos(w*t); U*sin(w*t); 0] - v_b_c; % velocity in body realtive to body in current
    v_n_b_relative(i+1,:) = (R_n_b * v_b_b_relative)';
    
    %relative to ocean
    v_b_b_c =  [U*cos(w*t); U*sin(w*t); 0]; % velcoity of body in body relativ to current
    v_n_b_c = R_n_b* v_b_b_c; % velocity of body in ned realtive to current
    
    % realtive to ocean with current
    v_n_c_n = R_n_b*v_b_c; % velcoity of current in ned realtive to ned
    v_n_b_n(i+1,:) = (v_n_b_c + v_n_c_n)'; % velcity of body in ned realtive to ned
    
    if i == 0
        p_n_b(i+1,:) = [0 0 0];
        p_n_b_c(i+1,:) = [0 0 0];
        p_n_current(i+1,:) = [0 0 0];
    else
        p_n_b(i+1,:) = p_n_b(i,:) + v_n_b(i,:)*h;
        p_n_b_c(i+1,:) = p_n_b_c(i,:) + v_n_b_n(i,:)*h;
        p_n_current(i+1,:) = p_n_current(i,:) + (v_n_c*h)';
    end
    
end

% plotting without current

speed = ( v_n_b(:,1).^2 + v_n_b(:,2).^2 + v_n_b(:,3).^2 ).^(1/2);
for i = 1:N
    crab_angle(i) = atan2(v_n_b(i,2),v_n_b(i,1)) .* rad2deg;
    sideslip_angle(i) = atan2(v_n_b(i,2),v_n_b(i,1)) .* rad2deg;
end
course_angle = (psi*rad2deg).*ones(N,1) + crab_angle ;

t = [0:h:t_end-1*h]';
figure_num = 1;

figure(figure_num)
plot(t, p_n_b); xlabel('s'),ylabel('m'), title('Distance from initial point');legend('x','y','z');

figure_num = figure_num + 1;
figure(figure_num)
plot(p_n_b(:,2), p_n_b(:,1)), xlabel('east [m]'),ylabel('north [m]'), title('Plot of vechicle position without current'), grid;

figure_num = figure_num + 1;
figure(figure_num)
subplot(211), plot(t, v_n_b), xlabel('s'), ylabel('m/s'),title('Relative velocities'); legend('u','v','w');
subplot(212), plot(t,speed), xlabel('s'), ylabel('m/s'), title('Speed');

figure_num = figure_num + 1;
figure(figure_num)
plot(t, crab_angle), hold on; 
plot(t,sideslip_angle)
plot(t,course_angle);title('Crab-, slip- and courseangle'); xlabel('t'); ylabel('deg'); legend('\beta', '\beta_r','\chi');

% plots with current
speed_relative = ( v_n_b_relative(:,1).^2 + v_n_b_relative(:,2).^2 + v_n_b_relative(:,3).^2 ).^(1/2);
for i = 1:N
    crab_angle(i) = atan2(v_n_b(i,2),v_n_b(i,1)) .* rad2deg;
    sideslip_angle(i) = atan2(v_n_b_relative(i,2),v_n_b_relative(i,1)) .* rad2deg;
end
course_angle = (psi*rad2deg).*ones(N,1) + crab_angle;

figure_num = figure_num + 1;
figure(figure_num)
plot(t, p_n_b_c); xlabel('s'),ylabel('m'), title('Distance from initial point with current');legend('x','y','z');

figure_num = figure_num + 1;
figure(figure_num)
plot(p_n_b_c(:,2), p_n_b_c(:,1)), xlabel('east [m]'),ylabel('north [m]'), title('Plot of vechicle position with current'), hold on;
plot(p_n_current(:,2), p_n_current(:,1),'-'); grid;

figure_num = figure_num + 1;
figure(figure_num)
subplot(211), plot(t, v_n_b_relative), xlabel('s'), ylabel('m/s'),title('Relative velocities with current'); legend('u','v','w');
subplot(212), plot(t,speed_relative), xlabel('s'), ylabel('m/s'), title('Speed');

figure_num = figure_num + 1;
figure(figure_num)
plot(t, crab_angle), hold on
plot(t,sideslip_angle), hold on
plot(t,course_angle), hold on
title('Crab-, slip- and courseangle with current'), xlabel('t'), ylabel('deg'), legend('\beta', '\beta_r','\chi');

