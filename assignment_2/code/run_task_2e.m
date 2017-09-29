close all;
clear all;
clc;

parameters_longitudal_autopilot; % get parameters
d = 0*deg2rad;
t_end = 100;


i = 1;
Legend = {};
figure(1);
for chi_c = [-5, 10, 15]*deg2rad
    model = 'lateral_autopilot.slx';
    load_system(model);
    simOut = sim(model);
    
    subplot(211)
    plot(chi.time,chi.signals.values.*rad2deg); hold on; 
    Legend(i) = {strcat('\chi_c = ', num2str(chi_c*rad2deg))};
    
    subplot(212)
    plot(delta_a_c.time,delta_a_c.signals.values.*rad2deg); hold on; axis([0 t_end -30 30]);
    i = i+1;
end
figure(1)
subplot(211)
legend(Legend);
subplot(212)
legend(Legend);

