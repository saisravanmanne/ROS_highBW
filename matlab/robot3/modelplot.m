function platoon = modelplot(L1,L2)
%% constant decleration
clc
close all 

ra1 = 4.641; % right motor
ra2 = 3.934; % left motor
Kg = 9.68;  % gear ratio
Kb = 0;
Kt = Kb;
%% load the proper csv file in here and name it as killerKb;
% Kb this is for back emf calculation 
l1 = L1;
l2 = L2; % size of the array
input_voltages = [11.87 11.39 11.09 10.70 10.10 9.09];
killerKb = csv2table('data.csv',l1,l2);

velocity = table2array(killerKb(:,2));
ang_velocity = table2array(killerKb(:,4));
ref_velocity = table2array(killerKb(:,6));
ref_ang_velocity = table2array(killerKb(:,8));
%% plot the step response
figure;
plot(l1:l2,velocity,'r');
xlabel({'Time','in milli-seconds (ms)'})
ylabel({'Angular Velocity (in rad/s) Linear Velocity (in m/s)'})
title('Robot1 charateristics')
hold on;
plot(l1:l2,ang_velocity)
plot(l1:l2,ref_velocity)
plot(l1:l2,ref_ang_velocity)
legend('Linear_velocity','Angular_velocity','Reference_Linear_Velocity','Reference_angular_velocity');







