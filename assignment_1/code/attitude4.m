function temp = atan2(y,x)
    if(x > 0)
        temp = atan(y./x);
    elseif(y>=0 and x<0)
        temp =pi/2 + atan(y./x);
    elseif(y<0 and x<0)
        temp = -pi/2 + atan(y./x);
    elseif(x == 0 and y > 0)
        temp =  pi/2;
   elseif(x == 0 and y < 0)
        temp =  -pi/2;
    elseif(x == 0 and y == 0)
        temp = -1000; % wrong
    end
end

%% Task 2.3
close all;
% some constants
deg2rad = pi/180;   
rad2deg = 180/pi;

% Namato constants
zeta_p  = 0.1; 
zeta_q  = 0.2; 
omega_p  = 0.1; 
omega_q  = 0.05; 

% turning rate parameteres
K_r = 0.1;
T_r = 50;

% Euler anlges init values
phi_0 = -1 *deg2rad;
theta_0 = 2.0 *deg2rad;
psi_0 = 0 * deg2rad;

%omega init values
p_0 = 0;
q_0 = 0;
r_0 = 0;

% current parameters
U_c = 0.6; % m/s
alpha_c = 10 * deg2rad;
beta_c = 45 * deg2rad;

v_n_c = [U_c*cos(alpha_c)*cos(beta_c);
         U_c*sin(beta_c);
         U_c*sin(alpha_c)*cos(beta_c)]; % Current velocity in ned

%% Simulation and plotting

t_end = 1500;
h = 0.1;
N = t_end/h;
U = 1.5;


v_n_b = zeros(N, 3);                    % velocities of body  realtive to ned in body coordinates                     
s_n_b = zeros(N, 3);                    % position of body realative to ned in ned coordinates
v_n_b_c = zeros(N, 3);                    % velocities of body  realtive to ned in body coordinates  with current                    
s_n_b_c = zeros(N, 3);                    % position of body realative to ned in ned coordinates with current
Theta = zeros(N, 3);                    % Euler angles 
rudder = zeros(N, 1);                    % Euler angles 
        

p = p_0;
q = q_0;
rudder(1) = r_0;
Theta(1,:) = [ phi_0 ,theta_0, psi_0];
p_n_b(1,:) = [0, 0, 0];
v_n_b(1,:) = [0, 0, 0];                    % velocities of body  realtive to ned in body coordinates                     
s_n_b(1,:) = [0, 0, 0];                    % position of body realative to ned in ned coordinates
v_n_b_c(1,:) = [0, 0, 0];                    % velocities of body  realtive to ned in body coordinates  with current                    
s_n_b_c(1,:) = [0, 0, 0];                    % position of body realative to ned in ned coordinates with current


for i = 1:N-1;
    t = i*h;

    w_b_b = [p; q; rudder(i)];
    
    [J,R,T] = eulerang(Theta(i,1),Theta(i,2),Theta(i,3));
    
    v_b_c = inv(R)*v_n_c;                               % current velocity in body    
    v_b_b = [U*cos(rudder(i)*t); U*sin(rudder(i)*t); 0];                % omega = r = yaw_rate
    v_b_b_c = v_b_b - v_b_c;      % omega = r = yaw_rate
    
    %preper for next iteration
    
    v_n_b(i+1,:) = (R * v_b_b)'; 
    v_n_b_c(i+1,:) = (R * v_b_b_c)'; 
    
    Theta_dot = (T * w_b_b);
    
    Theta(i+1,1:2) = Theta(i,1:2) + Theta_dot(1:2)'*h;
    Theta(i+1,3) = rudder(i);
    
    s_n_b(i+1,:) = s_n_b(i,:) + v_n_b(i,:)*h;
    s_n_b_c(i+1,:) = s_n_b_c(i,:) + v_n_b_c(i,:)*h;
    
    p_dot = -2*zeta_p*omega_p*p - omega_p^2*Theta(i,1);
    q_dot = -2*zeta_q*omega_q*q - omega_q^2*Theta(i,2);
    
    
    p = p + p_dot*h; 
    q = q + q_dot*h;
    
    if(t < 699)
        delta = 5*deg2rad;
    else
        delta = 10*deg2rad;
    end 
    
    rudder_dot= -(rudder(i)/T_r) + K_r/T_r*delta;
    rudder(i+1) = rudder(i) + rudder_dot*h;
      
end

% plotting without current

speed = ( v_n_b(:,1).^2 + v_n_b(:,2).^2 + v_n_b(:,3).^2 ).^(1/2);
speed_c = ( v_n_b_c(:,1).^2 + v_n_b_c(:,2).^2 + v_n_b_c(:,3).^2 ).^(1/2);
crab_angle = asin(v_n_b(:,2)./speed) .* rad2deg;
sideslip_angle = asin(v_n_b_c(:,2)./speed_c) .* rad2deg;
course_angle = (Theta(:,3)*rad2deg) + crab_angle ;

t = [0:h:t_end-1*h]';
figure_num = 0;

figure_num = figure_num + 1;
figure(figure_num)
plot(s_n_b(:,1), s_n_b(:,2)), xlabel('east [m]'),ylabel('north [m]'), title('Plot of vechicle position'), hold on;
plot(s_n_b_c(:,1), s_n_b_c(:,2)), legend('without current','with current'), grid;

figure_num = figure_num + 1;
figure(figure_num)
subplot(211), plot(t, v_n_b_c), xlabel('s'), ylabel('m/s'),title('Relative velocities'); legend('u','v','w');
subplot(212), plot(t,speed), xlabel('s'), ylabel('m/s'), title('Speed');

figure_num = figure_num + 1;
figure(figure_num)
subplot(111), plot(t, Theta), xlabel('s'), ylabel('m/s'),title('Attitude'); legend('\phi','\theta','\psi');

figure_num = figure_num + 1;
figure(figure_num)
plot(t, crab_angle), hold on; 
plot(t,sideslip_angle)
plot(t,course_angle);title('Crab-, slip- and courseangle'); xlabel('t'); ylabel('deg'); legend('\beta', '\beta_r','\chi');

figure_num = figure_num + 1;
figure(figure_num)
subplot(211), plot(t, v_n_b_c), xlabel('s'), ylabel('m/s'),title('Relative velocities with current'); legend('u','v','w');
subplot(212), plot(t,speed_c), xlabel('s'), ylabel('m/s'), title('Speed');

