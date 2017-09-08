%% Task 2.3
close all;
clc;
% some constants
deg2rad = pi/180;   
rad2deg = 180/pi;

% Namato constants
zeta_p  = 0.1; 
zeta_q  = 0.2; 
omega_p  = 0.1; 
omega_q  = 0.05; 

% turning rate parameteres
K_n = 0.1;
T_n = 50;

%omega init values
p_0 = 0;
q_0 = 0;
r_0 = 0;

% Euler anlges init values
phi_0 = -1 *deg2rad;
theta_0 = 2.0 *deg2rad;
psi_0 = 0 * deg2rad;

% current parameters
U_c = 0.6; % m/s
alpha_c = 10 * deg2rad;
beta_c = 45 * deg2rad;

v_n_c = [U_c*cos(alpha_c)*cos(beta_c);
         U_c*sin(beta_c);
         U_c*sin(alpha_c)*cos(beta_c)]; % Current velocity in ned
     
           
%% Task 2.4   Simulation and plotting

% Without current
t_end = 1400;
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
Theta = zeros(N, 3);                     % Euler angles 
rudder = zeros(N, 1);                    % Euler angles 


p = p_0;
q = q_0;
rudder(1) = r_0;
Theta(1,:) = [ phi_0 ,theta_0, psi_0];

for i = 1:N-1;
    
    t = (i-1)*h;
    
    [J,R,T] = eulerang(Theta(i,1),Theta(i,2),Theta(i,3));
    
    w_b_b = [p; q; rudder(i)];
    Theta_dot = (T * w_b_b);
    Theta(i+1,1:2) = Theta(i,1:2) + Theta_dot(1:2)'*h;
    Theta(i+1,3) = rudder(i);
    
    %without current
    v_b_b = [U*cos(rudder(i)*t); U*sin(rudder(i)*t); 0];
    v_n_b(i+1,:) = (R_n_b * v_b_b)';
    
    % relative velocity
    v_b_b_relative = [U*cos(rudder(i)*t); U*sin(rudder(i)*t); 0] - v_b_c; % velocity in body realtive to body in current
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
    
    p_dot = -2*zeta_p*omega_p*p - omega_p^2*Theta(i,1);
    q_dot = -2*zeta_q*omega_q*q - omega_q^2*Theta(i,2);
    
    p = p + p_dot*h; 
    q = q + q_dot*h;
    
    if(t < 699)
        delta = 5*deg2rad;
    else
        delta = 10*deg2rad;
    end 
    
    rudder_dot = -(rudder(i)/T_n) + K_n/T_n*delta;
    rudder(i+1) = rudder(i) + rudder_dot*h;
    
end

% plotting without current

speed = ( v_n_b(:,1).^2 + v_n_b(:,2).^2 + v_n_b(:,3).^2 ).^(1/2);
speed_relative = ( v_n_b_relative(:,1).^2 + v_n_b_relative(:,2).^2 + v_n_b_relative(:,3).^2 ).^(1/2);
for i = 1:N
    crab_angle(i) = atan2(v_n_b(i,2),v_n_b(i,1)) .* rad2deg;
    sideslip_angle(i) = atan2(v_n_b_relative(i,2),v_n_b_relative(i,1)) .* rad2deg;
end
course_angle = (Theta(:,3)*rad2deg) + crab_angle ;


t = [0:h:t_end-1*h]';
figure_num = 0;

figure_num = figure_num + 1;
figure(figure_num), axis tight equal;
plot(p_n_b(:,2), p_n_b(:,1)), hold on;
plot(p_n_b_c(:,2), p_n_b_c(:,1)), xlabel('east [m]'),ylabel('north [m]'), title('Plot of vechicle position without current'), grid;

figure_num = figure_num + 1;
figure(figure_num), axis tight equal;
subplot(211), plot(t, v_n_b_relative), xlabel('s'), ylabel('m/s'),title('Relative velocities'); legend('u','v','w');
subplot(212), plot(t,speed_relative), xlabel('s'), ylabel('m/s'), title('Speed');

figure_num = figure_num + 1;
figure(figure_num), axis tight equal;
plot(t, crab_angle), hold on; 
plot(t,sideslip_angle)
plot(t,course_angle);title('Crab-, slip- and courseangle'); xlabel('t'); ylabel('deg'); legend('\beta', '\beta_r','\chi');





