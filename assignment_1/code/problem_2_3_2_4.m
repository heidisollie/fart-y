
clear all;
close all;

% some constants
deg2rad = pi/180;   
rad2deg = 180/pi;

% euler angles
phi = 0;
theta = 2.0*deg2rad;
psi = 30*deg2rad;

% current parameters
U_c = 0.6; % m/s
alpha_c = 10 * deg2rad;
beta_c = 45 * deg2rad;

v_n_c_n = [U_c*cos(alpha_c)*cos(beta_c);
         U_c*sin(beta_c);
         U_c*sin(alpha_c)*cos(beta_c)]; % Current velocity in ned

v_b_b_c = [1.5;  0; 0];

R_n_b = Rzyx(phi,theta,psi); 

%% calculating sideslip angle
% withcurrent
v_b_r = v_b_b_c - inv(R_n_b)*v_n_c_n; % velocity realtive for vechicle ( current moving faster)
U_r = norm(v_b_r)
sideslip_angle = asin(v_b_r(2)/U_r)*rad2deg

% without current
v_b_r = v_b_b_c;
U_r = norm(v_b_r);
sideslip_angle = asin(v_b_r(2)/U_r)*rad2deg

%% Task 2.4   Simulation and plotting

% Without current
t_end = 419*3;
h = 0.1;
N = t_end/h;

p_n_b = zeros(N, 3);                  % position of body with current realative to ned in ned coordinates
p_n_b_cur = zeros(N, 3);                  % position of body with current realative to ned in ned coordinates


for i = 0:N-1;
    v_b_r = v_b_b_c ; %(R_n_b)'*v_n_c_n; % velocity realtive for vechicle ( current moving faster)
    v_n_b = (R_n_b * v_b_b_c);
    v_n_b_cur = v_n_b + v_n_c_n;

    if i == 0
        p_n_b(i+1,:) = [0 0 0];
    else
        p_n_b(i+1,:) = p_n_b(i,:) + v_n_b'*h;
        p_n_b_cur(i+1,:) = p_n_b_cur(i,:) + v_n_b_cur'*h;
    end
    
end

figure(1), axis tight equal;
plot(p_n_b_cur(:,2), p_n_b_cur(:,1)),  hold on;
plot(p_n_b(:,2), p_n_b(:,1)), xlabel('east [m]'),ylabel('north [m]'), title('Plot of vechicle position without current'),legend('current','not current')