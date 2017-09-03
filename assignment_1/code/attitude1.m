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
h = 0.1;                     % sample time (s)
N  = 2000;                    % number of samples

% model parameters
m = 100;
r = 2.0;
I = diag([m*r^2 m*r^2 m*r^2]);       % inertia matrix
I_inv = inv(I);

%controll parameters
k_p = 1;
k_d = 20;
K = [   0 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0;
        k_p 0 0 k_d 0 0;
        0 k_p 0 0 k_d 0;
        0 0 k_p 0 0 k_d];

% constants
deg2rad = pi/180;   
rad2deg = 180/pi;

phi = 10*deg2rad;            % initial Euler angles
theta = -5*deg2rad;
psi = 15*deg2rad;

q = euler2q(phi,theta,psi);   % transform initial Euler angles to q

w = [0 0 0]';                 % initial angular rates

table = zeros(N+1,14);        % memory allocation

%% FOR-END LOOP
for i = 1:N+1,
   t = (i-1)*h;                  % time
   u = -K*[q(2:end)' w']';       % control law
   tau = u(4:end);               % u is for 6 states, tau is fore 3 states

   [phi,theta,psi] = q2euler(q); % transform q to Euler angles
   [J,J1,J2] = quatern(q);       % kinematic transformation matrices
   
   q_dot = J2*w;                        % quaternion kinematics
   w_dot = I_inv*(Smtrx(I*w)*w + tau);  % rigid-body kinetics
   
   table(i,:) = [t q' phi theta psi w' tau'];  % store data in table
   
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

clf
figure(1)
subplot(311),plot(t,phi),xlabel('time (s)'),ylabel('deg'),title('\phi'),grid
subplot(312),plot(t,theta),xlabel('time (s)'),ylabel('deg'),title('\theta'),grid
subplot(313),plot(t,psi),xlabel('time (s)'),ylabel('deg'),title('\psi'),grid
figure(3)
subplot(211),plot(t,q),xlabel('time (s)'),ylabel('rad'),title('q'),grid, legend('\eta', '\epsilon_1', '\epsilon_2', '\epsilon_3')
subplot(212),plot(t,w),xlabel('time (s)'),ylabel('deg/s'),title('w'),grid, legend('w_1', 'w_2', 'w_3')
figure(2)
subplot(111),plot(t,tau),xlabel('time (s)'),ylabel('Nm'),title('\tau'),grid, legend('\tau_1', '\tau_2', '\tau_3')