%% Task 2.3
close all;
clear all;
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


v_n_c_n = [U_c*cos(alpha_c)*cos(beta_c);
         U_c*sin(beta_c);
         U_c*sin(alpha_c)*cos(beta_c)]; % Current velocity in ned
     
v_b_c_n = inv(R_n_b)*v_n_c_n;               % current velocity in body             

%% Task 2.4   Simulation and plotting

% Without current
t_end = 419*3;
h = 0.1;
N = t_end/h;
U = 1.5;
w = 0.859 * deg2rad ;                   % turning rate  of vechicle based on circular motion

v_n_b_c = zeros(N, 3);                    % velocities of body  realtive to ned in body coordinates                     
p_n_b = zeros(N, 3);                    % position of body relative to ned in ned coordinates, no current
v_n_b_relative = zeros(N, 3);                  % velocities of body with current  realtive to ned in body coordinates
p_n_b_cur = zeros(N, 3);                  % position of body with current relative to ned in ned coordinates
v_n_b_n =zeros(N, 3); 
p_n_current = zeros(N,3);                  % position of a point riding on the current

crab_angle = zeros(N,1);
sideslip_angle = zeros(N,1);
course_angle_cur = zeros(N,1); % with current
course_angle = zeros(N,1); % without current


%[J,R_nb,T] = eulerang(phi,theta,psi);

for i = 1:N;
    t = i*h;
    
    %Relative to ocean/current
    v_b_b_c = [U*cos(w*t); U*sin(w*t); 0];
    v_n_b_c(i,:) = (R_n_b * v_b_b_c)'; % velcoity of body in body relativ to current
    
    % relative velocity
    %v_b_b_relative = [U*cos(w*t); U*sin(w*t); 0] - v_b_c_n; % velocity in body realtive to body in current
    %v_n_b_relative(i+1,:) = (R_n_b * v_b_b_relative)';
    
    %relative to ocean
    %v_b_b_c =  [U*cos(w*t); U*sin(w*t); 0]; % velcoity of body in body relativ to current
    %v_n_b_c = R_n_b* v_b_b_c; % velocity of body in ned realtive to current
    
    % realtive to ned
    v_n_c_n = R_n_b*v_b_c_n; % velcoity of current in ned realtive to ned
    v_n_b_n(i,:) = (v_n_b_c(i,:) + v_n_c_n'); % velcity of body in ned realtive to ned
    
    if i == N
        %do nothing, you're finished here
    else
        p_n_b(i+1,:) = p_n_b(i,:) + v_n_b_c(i,:)*h; % no current
        p_n_b_cur(i+1,:) = p_n_b_cur(i,:) + v_n_b_n(i,:)*h; % with current
        p_n_current(i+1,:) = p_n_current(i,:) + (v_n_c_n*h)'; % This is the current itself
    end
    
    crab_angle(i) = atan2(v_n_b_n(i,2),v_n_b_n(i,1)) + pi;
    sideslip_angle(i) = atan2(v_b_b_c(2),v_b_b_c(1)) + pi; 
    course_angle(i) = mod((psi + sideslip_angle(i)),2*pi); % when there is no current crab=sideslip.
    course_angle_cur(i) = mod((psi + crab_angle(i)),2*pi); 
    
end

figure_num = 0;
t = [0:h:t_end-1*h]';
% plotting without current

figure_num = figure_num + 1;
figure(figure_num); 
plot(p_n_b(:,2), p_n_b(:,1)), xlabel('east [m]'),ylabel('north [m]'), title('Plot of vehicle position in NED, without current'), grid; axis equal;

figure_num = figure_num + 1;
figure(figure_num)
plot(t, sideslip_angle*rad2deg), hold on; 
plot(t,sideslip_angle*rad2deg);
plot(t,course_angle_cur*rad2deg);title('Crab-, slip- and courseangle given in body, without current'); xlabel('t'); ylabel('deg'); legend('\beta', '\beta_r','\chi');

% Plotting with current

speed = ( v_n_b_n(:,1).^2 + v_n_b_n(:,2).^2 + v_n_b_n(:,3).^2 ).^(1/2);

figure_num = figure_num + 1;
figure(figure_num)
plot(p_n_b_cur(:,2), p_n_b_cur(:,1)), xlabel('east [m]'),ylabel('north [m]'), title('Plot of vehicle position in NED with current'), hold on;
plot(p_n_current(:,2), p_n_current(:,1),'-'); grid;

figure_num = figure_num + 1;
figure(figure_num)
subplot(211), plot(t, v_n_b_n), xlabel('s'), ylabel('m/s'),title('Velocities relative to NED, with current'); legend('u','v','w');
subplot(212), plot(t,speed), xlabel('s'), ylabel('m/s'), title('Speed');

speed_relative = ( v_n_b_c(:,1).^2 + v_n_b_c(:,2).^2 + v_n_b_c(:,3).^2 ).^(1/2);

figure_num = figure_num + 1;
figure(figure_num)
subplot(211), plot(t, v_n_b_c), xlabel('s'), ylabel('m/s'),title('Velocities relative to current'); legend('u','v','w');
subplot(212), plot(t,speed_relative), xlabel('s'), ylabel('m/s'), title('Speed');

figure_num = figure_num + 1;
figure(figure_num)
plot(t, crab_angle*rad2deg), hold on; 
plot(t,sideslip_angle*rad2deg)
plot(t,course_angle_cur*rad2deg)
title('Crab-, slip- and courseangle in body with current'), xlabel('t'), ylabel('deg'), legend('\beta', '\beta_r','\chi');

