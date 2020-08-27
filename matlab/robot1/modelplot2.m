function platoon = modelplot2(L1,L2)
%% constant decleration
clc
%close all 

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
killerKb = csv2table('cruiseData.csv',l1,l2);

position_x = table2array(killerKb(:,2));
position_y = table2array(killerKb(:,4));
theta = table2array(killerKb(:,6));
linear_vel = table2array(killerKb(:,8));
angular_vel = table2array(killerKb(:,10));
time = table2array(killerKb(:,12));
Theta_ref = table2array(killerKb(:,14));
V_ref = table2array(killerKb(:,16));
%% plot the step response
figure; 
plot(position_x,position_y);
grid on;
xlabel({'time'})
ylabel({'Wheel Angular Velocity'})
title('MotorShit')
legend('linear_vel','angular_vel');
figure;
plot(time,V_ref);
grid on;
xlabel({'time'});
ylabel({'linear & angular vel'});
title('Hah Hah');
%legend('linear_vel','angular_vel')