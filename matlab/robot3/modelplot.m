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
td = 1/90;
input_voltages = [11.87 11.39 11.09 10.70 10.10 9.09];
killerKb = csv2table('data.csv',l1,l2);

velocity = table2array(killerKb(:,2));
ang_velocity = table2array(killerKb(:,4));
ref_velocity = table2array(killerKb(:,6));
ref_ang_velocity = table2array(killerKb(:,8));
%% plot the step response
figure;
plot((l1:l2)*td,velocity,'r');
xlabel({'Time (s)'})
ylabel({'Angular Velocity (in rad/s)'})
title('Robot3 charateristics')
hold on;
plot((l1:l2)*td,ang_velocity,'g')
plot((l1:l2)*td,-ref_velocity,'b')
plot((l1:l2)*td,-ref_ang_velocity,'b')
legend('AngVel Right','AngVel Left','Ref AngVel Right','Ref AngVel Left');







