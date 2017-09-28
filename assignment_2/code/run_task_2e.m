close all;
clear all;
clc;

parameters_longitudal_autopilot; % get parameters
%change !!!!!
chi_c  = 10*deg2rad;
d = 8*deg2rad;


i = 1;
Legend = {};
for chi_c = [-5, 10, 15]*deg2rad
    model = 'lateral_autopilot.slx';
    load_system(model);
    simOut = sim(model);
    figure(1);
    plot(chi.time,chi.signals.values.*rad2deg); hold on;
    Legend(i) = {strcat('\chi_c = ', num2str(chi_c*rad2deg))};
    figure(2);
    plot(delta_a_c.time,delta_a_c.signals.values.*rad2deg); hold on;
    i = i+1;
end
figure(1)
legend(Legend);
figure(2)
legend(Legend);

