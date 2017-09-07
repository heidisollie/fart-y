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

v = zeros(N, 3); % velocities 
s = zeros(N, 3); % posisjon


for i = 0:N-1;
    t = i*h;
    v(i+1,:) = [U*cos(w*t) U*sin(w*t) 0];
    if i == 0
        s(i+1,1) = 0;
        s(i+1,2) = 0;
        s(i+1,3) = 0;
    else
        s(i+1,1) = s(i,1) + v(i,1)*h;
        s(i+1,2) = s(i,2) + v(i,2)*h;
        s(i+1,3) = s(i,3) + v(i,3)*h;
    end
end
speed = ( v(:,1).^2 + v(:,2).^2 + v(:,3).^2 ).^(1/2);
crab_angle = arcsin(v(:,2)/speed);
sideslip_angle = arcsin(v(:,2)/speed);
course_angle = yaw + sideslip_angle;

t = [0:h:t_end-1*h]';
figure(figure_num)
plot(t, s); title('Distance from initial point');legend('x','y','z');

figure_num = figure_num + 1;
figure(figure_num)
plot(s(:,1), s(:,2)); title('Plot of vechicle position without current');

figure_num = figure_num + 1;
figure(figure_num)
subplot(211); plot(t, v);title('Relative velocities'); legend('u','v','w');
subplot(212); plot(t,speed);title('Speed');

figure_num = figure_num + 1;
figure(figure_num)
subplot(311); plot(t, crab_angle);title('Crab_angle'); legend('u','v','w');
subplot(312); plot(t,sideslip_angle);title('Sideslip angle');
subplot(312); plot(t,course_angle);title('Course angle');





% With current
v = zeros(N, 3); % velocities 
s = zeros(N, 3); % posisjon
t_end = t_end *3;
N = t_end/h;
for i = 0:N-1;
    t = i*h;
    v(i+1,:) = [U*cos(w*t) U*sin(w*t) 0] - v_b_c';
    if i == 0
        s(i+1,1) = 0;
        s(i+1,2) = 0;
        s(i+1,3) = 0;
    else
        s(i+1,1) = s(i,1) + v(i,1)*h;
        s(i+1,2) = s(i,2) + v(i,2)*h;
        s(i+1,3) = s(i,3) + v(i,3)*h;
    end
end
   
figure_num = figure_num + 1;
figure(figure_num)
t = [0:h:t_end - h];
plot(t, s); title('Distance from initial point with current'); legend('x', 'y', 'z');

figure_num = figure_num + 1;
figure(figure_num)
plot(s(:,1), s(:,2)); title('Plot of vechivle position with current');

