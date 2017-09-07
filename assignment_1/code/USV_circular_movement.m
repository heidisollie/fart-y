%% Task 2.3
phi = 0 *deg2rad;
theta = 2.0 *deg2rad;
psi = 30 * deg2rad;

U_c = 0.6; % m/s
alpha_c = 10 * deg2rad;
beta_c = 45 * deg2rad;

figure_num = 1;

R_n_b = Rzyx(phi,theta,psi);
v_n_c = [U_c*cos(alpha_c)*cos(beta_c);
         U_c*sin(beta_c);
         U_c*sin(alpha_c)*cos(beta_c)];
     
v_b_c = inv(R_n_b)*v_n_c

%% Task 2.4

deg2rad = pi/180;   
rad2deg = 180/pi;
% Without current
t_end = 419;
h = 0.1;
N = t_end/h;
U = 1.5;
w = 0.859 * deg2rad ;

v_n_b = zeros(N, 3); % velocities of body  realtive to ned in body coordinates
p_n_b = zeros(N, 3); % position of body realative to ned in ned coordinates

[J,R_nb,T] = eulerang(phi,theta,psi);

for i = 0:N-1;
    t = i*h;
    v_b_b = [U*cos(w*t); U*sin(w*t); 0];
    v_n_b(i+1,:) = (R_nb * v_b_b)';
    if i == 0
        s(i+1,:) = [0 0 0];
    else
        s(i+1,:) = s(i,:) + v_n_b(i,:)*h;
    end
end
speed = ( v_n_b(:,1).^2 + v_n_b(:,2).^2 + v_n_b(:,3).^2 ).^(1/2);
crab_angle = asin(v_n_b(:,2)/speed) * rad2deg;
sideslip_angle = asin(v_n_b(:,2)/speed) * rad2deg;
course_angle = (psi *rad2deg) + sideslip_angle ;

t = [0:h:t_end-1*h]';
figure(figure_num)
plot(t, s); xlabel('s'),ylabel('m'), title('Distance from initial point');legend('x','y','z');

figure_num = figure_num + 1;
figure(figure_num)
plot(s(:,1), s(:,2)), xlabel('east [m]'),ylabel('north [m]'), title('Plot of vechicle position without current'), grid;

figure_num = figure_num + 1;
figure(figure_num)
subplot(211), plot(t, v_n_b), xlabel('s'), ylabel('m/s'),title('Relative velocities'); legend('u','v','w');
subplot(212), plot(t,speed), xlabel('s'), ylabel('m/s'), title('Speed');

figure_num = figure_num + 1;
figure(figure_num)
plot(t, crab_angle), hold on; 
plot(t,sideslip_angle)
plot(t,course_angle);title('Crab-, slip- and courseangle'), xlabel('t'), ylabel('deg'), legend('\beta', '\beta_r','\khi');

% With current
v_n_b = zeros(N, 3); % velocities 
s = zeros(N, 3); % posisjon
t_end = t_end *3;
N = t_end/h;
for i = 0:N-1;
    t = i*h;
    
    v_b_b = [U*cos(w*t); U*sin(w*t) 0] - v_b_c;
    
    v_n_b(i+1,:) = (R_nb * v_b_b)';
    if i == 0
        s(i+1,:) = [0 0 0];
    else
        s(i+1,:) = s(i,:) + v_n_b(i,:)*h;
    end
end
   
speed = ( v_n_b(:,1).^2 + v_n_b(:,2).^2 + v_n_b(:,3).^2 ).^(1/2);
crab_angle = asin(v_n_b(:,2)/speed) * rad2deg;
sideslip_angle = asin(v_n_b(:,2)/speed) * rad2deg;
course_angle = (psi + sideslip_angle) * rad2deg;

t = [0:h:t_end-1*h]';
figure(figure_num)
plot(t, s); xlabel('s'),ylabel('m'), title('Distance from initial point with current');legend('x','y','z');

figure_num = figure_num + 1;
figure(figure_num)
plot(s(:,1), s(:,2)), xlabel('east [m]'),ylabel('north [m]'), title('Plot of vechicle position with current'), grid;

figure_num = figure_num + 1;
figure(figure_num)
subplot(211), plot(t, v_n_b), xlabel('s'), ylabel('m/s'),title('Relative velocities with current'); legend('u','v','w');
subplot(212), plot(t,speed), xlabel('s'), ylabel('m/s'), title('Speed');

figure_num = figure_num + 1;
figure(figure_num)
plot(t, crab_angle), hold on; 
plot(t,sideslip_angle)
plot(t,course_angle);title('Crab-, slip- and courseangle with current'), xlabel('t'), ylabel('deg'), legend('\beta', '\beta_r','\khi');

