% M-script for numerical integration of the attitude dynamics of a rigid 
% body represented by unit quaternions. The MSS m-files must be on your
% Matlab path in order to run the script.
%
% System:                      .
%                              q = T(q)w
%                              .
%                            I w - S(Iw)w = tau
% Control law:
%                            tau = constant
% 
% Definitions:             
%                            I = inertia matrix (3x3)
%                            S(w) = skew-symmetric matrix (3x3)
%                            T(q) = transformation matrix (4x3)
%                            tau = control input (3x1)
%                            w = angular velocity vector (3x1)
%                            q = unit quaternion vector (4x1)
%
% Author:                   2016-05-30 Thor I. Fossen 

%% USER INPUTS
h = 0.1;                               % sample time (s)
N  = 2000;                             % number of samples

% model parameters
m = 100;
r = 2.0;
I = diag([m*r^2 m*r^2 m*r^2]);         % inertia matrix
I_inv = inv(I);

%controll parameters
k_p = 10;
k_d = 300;
K = [   0 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0;
        k_p 0 0 k_d 0 0;
        0 k_p 0 0 k_d 0;
        0 0 k_p 0 0 k_d];

% constants
deg2rad = pi/180;   
rad2deg = 180/pi;

% initialization
phi = 10*deg2rad;                  % initial Euler angles
theta = -5*deg2rad;
psi = 15*deg2rad;

q = euler2q(phi,theta,psi);        % transform initial Euler angles to q

w = [0 0 0]';                      % initial angular rates

table = zeros(N+1,14);             % memory allocation
tracking_error = zeros(N+1,3);     % memory allocation

%% FOR-END LOOP
for i = 1:N+1,
   t = (i-1)*h;                           % time      
   
   phi_d =10*sin(0.1*t)*deg2rad;          % calculate desired values in euler angles
   theta_d = 0*deg2rad;
   psi_d = 15*cos(0.05*t)*deg2rad;
   
   q_d = euler2q(phi_d,theta_d,psi_d);    % calculate desired values in quaternions
   
   q_d_conj = quatconj(q_d');
   q_tilde = quatmultiply(q', q_d_conj)'; % calculate error in quaternion coordinates
   
   u = -K*[q_tilde(2:end)' w']';          % control law
   tau = u(4:end);                        % u is input for 6 states, tau is input for 3 states

   [phi,theta,psi] = q2euler(q);          % transform q to Euler angles
   [J,J1,J2] = quatern(q);                % kinematic transformation matrices
   
   q_dot = J2*w;                          % quaternion kinematics
   w_dot = I_inv*(Smtrx(I*w)*w + tau);    % rigid-body kinetics
   
   table(i,:) = [t q' phi theta psi w' tau'];           % store data in table
   desired_euler_angles(i,:) = [phi_d theta_d psi_d];   % store desired values in table
   
   q = q + h*q_dot;	             % Euler integration
   w = w + h*w_dot;
   
   q  = q/norm(q);               % unit quaternion normalization
   
end 

%% PLOT FIGURES
t       = table(:,1);  
q       = table(:,2:5); 
phi     = rad2deg*table(:,6);
theta   = rad2deg*table(:,7);
psi     = rad2deg*table(:,8);
w       = rad2deg*table(:,9:11);  
tau     = table(:,12:14);
phi_d   = rad2deg*( desired_euler_angles(:,1));
theta_d = rad2deg*( desired_euler_angles(:,2));
psi_d   = rad2deg*( desired_euler_angles(:,3));
phi_error   = phi -  phi_d ;    % error = tracking_error
theta_error = theta - theta_d;
psi_error   = psi - psi_d ;

clf
figure(1)
subplot(411),plot(t,phi),hold on, plot(t,phi_d),xlabel('time (s)'),ylabel('deg'),title('\phi'),grid, legend('\phi', '\phi_d')
subplot(412),plot(t,theta),hold on,plot(t,theta_d),xlabel('time (s)'),ylabel('deg'),title('\theta'),grid, legend('\theta', '\theta_d')
subplot(413),plot(t,psi),hold on, plot(t,psi_d),xlabel('time (s)'),ylabel('deg'),title('\psi'),grid, legend('\psi', '\psi_d')
subplot(414),plot(t,w),xlabel('time (s)'),ylabel('deg/s'),title('w'),grid,legend('w_1', 'w_2', 'w_3')

figure(2)
subplot(411),plot(t,tau),xlabel('time (s)'),ylabel('Nm'),title('\tau'),grid, legend('\tau_1', '\tau_2', '\tau_3')
subplot(412),plot(t,phi_error),xlabel('time (s)'),ylabel('deg'),title('tracking error \phi'),grid
subplot(413),plot(t,theta_error),xlabel('time (s)'),ylabel('deg'),title('tracking error \theta '),grid
subplot(414),plot(t,psi_error),xlabel('time (s)'),ylabel('deg'),title('tracking error \psi'),grid