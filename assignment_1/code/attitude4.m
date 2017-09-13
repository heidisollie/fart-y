%% Task 2.3
close all;
clc;
clear all;
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

v_n_c_n = [U_c*cos(alpha_c)*cos(beta_c);
         U_c*sin(beta_c);
         U_c*sin(alpha_c)*cos(beta_c)]; % Current velocity in ned
     
           
%% Task 2.4   Simulation and plotting

% Without current
t_end = 1500;
h = 0.1;
N = t_end/h;
U = 1.5;

v_n_b_c = zeros(N, 3);                    % velocities of body realtive to current in ned coordinates                     
p_n_b = zeros(N, 3);                    % position of body in ned coordinates
v_b_b_c = zeros(N, 3);                  % velocities of body relative to current given in body
p_n_b_cur = zeros(N, 3);                  % position of body with current in ned coordinates
v_n_b_n =zeros(N, 3);                   % Velocity of body relative to NED given in NED
p_n_current = zeros(N,3);               % The actual current
w_b_b_n = zeros(N,3);                   % angular velocity of body relative to ned, given in body

crab_angle = zeros(N,1);
sideslip_angle = zeros(N,1);
Theta = zeros(N, 3);                     % Euler angles 
r = zeros(N, 1);                    % Euler angles 
course_angle = zeros(N, 1);

t_shift_rudder = 700;

p = p_0;
q = q_0;
r(1) = r_0;
Theta(1,:) = [ phi_0 ,theta_0, psi_0];


for i = 1:N-1
    
    t = (i-1)*h;
    
    [J,R_n_b,T] = eulerang(Theta(i,1),Theta(i,2),Theta(i,3));
    
    w_b_b_n(i,:) = [p q r(i)]';
    Theta_dot = (T * w_b_b_n(i,:)');
    
    v_b_c_n = R_n_b'*v_n_c_n;
   
    %relative to ocean
    v_b_b_c(i,:) =  [U*cos(r(i)*t); U*sin(r(i)*t); 0]; 
    v_n_b_c(i,:) = (R_n_b* v_b_b_c(i,:)')'; 
    
    % realtive to ned with current
    v_n_b_n(i,:) = (v_n_b_c(i,:) + v_n_c_n'); 
    
    % Prep for next iteration
    p_n_b(i+1,:) = p_n_b(i,:) + v_n_b_c(i,:)*h; % no current
    p_n_b_cur(i+1,:) = p_n_b_cur(i,:) + v_n_b_n(i,:)*h; % with current
    p_n_current(i+1,:) = p_n_current(i,:) + (v_n_c_n*h)'; % //the// current
    
    p_dot = -2*zeta_p*omega_p*p - omega_p^2*Theta(i,1);
    q_dot = -2*zeta_q*omega_q*q - omega_q^2*Theta(i,2);
    
    p = p + p_dot*h; 
    q = q + q_dot*h;
    
    if(t < t_shift_rudder)
        delta = 5*deg2rad;
    else
        delta = 10*deg2rad;
    end 
    
    r_dot = -(r(i)/T_n) + (K_n/T_n)*delta;
    r(i+1) = r(i) + r_dot*h;
    
    Theta(i+1,:) = Theta(i,:) + Theta_dot(:)'*h;
    Theta(i+1,3) = mod(Theta(i+1,3),2*pi);
    
    v_b_b_n = R_n_b'*v_n_b_n(i,:)';
    
    crab_angle(i) = atan2(v_b_b_n(2),v_b_b_n(1)) + pi; 
    sideslip_angle(i) = atan2(v_b_b_c(i,2),v_b_b_c(i,1)) + pi;
    course_angle(i) = mod((Theta(i,3) + crab_angle(i)),2*pi); 
end

% plotting without current

speed = ( v_n_b_n(:,1).^2 + v_n_b_n(:,2).^2 + v_n_b_n(:,3).^2 ).^(1/2);
speed_relative = ( v_b_b_c(:,1).^2 + v_b_b_c(:,2).^2 + v_b_b_c(:,3).^2 ).^(1/2);

t = [0:h:t_end-1*h]';
figure_num = 0;

figure_num = figure_num + 1;
figure(figure_num), axis equal;
plot(p_n_b(:,2), p_n_b(:,1)), hold on; xlabel('east [m]'),ylabel('north [m]'), title('Plot of vechicle position without current'), grid;

figure_num = figure_num + 1;
figure(figure_num), axis tight equal;
plot(p_n_b_cur(:,2), p_n_b_cur(:,1)); xlabel('east [m]'),ylabel('north [m]'), title('Plot of vechicle position with current'), grid;

figure_num = figure_num + 1;
figure(figure_num), axis tight equal;
subplot(211), plot(t, v_n_b_c), xlabel('s'), ylabel('m/s'),title('Velocities relative to current'); legend('u','v','w');
subplot(212), plot(t,speed_relative), xlabel('s'), ylabel('m/s'), title('Speed');

figure_num = figure_num + 1;
figure(figure_num), axis tight equal;
subplot(211), plot(t, v_n_b_n), xlabel('s'), ylabel('m/s'),title('Velocities relative to NED');  legend('u','v','w');
subplot(212), plot(t,speed), xlabel('s'), ylabel('m/s'), title('Speed');

figure_num = figure_num + 1;
figure(figure_num), axis tight equal;
plot(t, crab_angle*rad2deg), hold on; 
plot(t,sideslip_angle*rad2deg)
plot(t,course_angle*rad2deg);title('Crab-, slip- and course angle with current'); xlabel('t'); ylabel('deg'); legend('\beta', '\beta_r','\chi');

figure_num = figure_num + 1;
figure(figure_num), axis tight equal;
plot(t,Theta*rad2deg);title('Roll, pitch , yaw with current'); xlabel('t'); ylabel('deg'); legend('\phi', '\theta','\psi');

figure_num = figure_num + 1;
figure(figure_num), axis tight equal;
plot(t, w_b_b_n*rad2deg); title('Angular velocities with current'); legend('p', 'q','r'), hold on; 







