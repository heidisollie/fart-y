close all;
clear all;
clc;

parameters_longitudal_autopilot; % get parameters
%change !!!!!
chi_c  = 10*deg2rad;
d = 0*deg2rad;
t_end = 100;


i = 1;
figure(1)
Legend = {};
for chi_c = [-5, 10, 15]*deg2rad
    model = 'lateral_autopilot_full.slx';
    load_system(model);
    simOut = sim(model);
    subplot(211)
    plot(chi.time,chi.signals.values.*rad2deg); hold on; 
    Legend(i) = {strcat('\chi_c = ', num2str(chi_c*rad2deg))};
    
    subplot(212)
    plot(delta_a_c.time,delta_a_c.signals.values.*rad2deg); hold on; axis([0 t_end -35 35]);
    i = i+1;
end
figure(1)
subplot(211)
legend(Legend);
subplot(212)
legend(Legend);