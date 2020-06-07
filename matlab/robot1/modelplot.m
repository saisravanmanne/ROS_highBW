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
killerKb = csv2table('arduino.csv',l1,l2);

right = table2array(killerKb(:,2));
left = table2array(killerKb(:,4));
[v,w] = fKin(right,left);
time = table2array(killerKb(:,6));
servo = table2array(killerKb(:,8));
linear_vel = table2array(killerKb(:,10));
angular_vel = table2array(killerKb(:,12));

%% plot the step response
figure; 
plot(time,right,time,servo);
grid on;
xlabel({'time (microsec)'})
ylabel({'Wheel Angular Velocity'})
title('MotorShit')
legend('right','left');
figure;
plot(time, left, time, servo);
grid on;
xlabel({'time (microsec)'});
ylabel({'linear & angular vel'});
title('Hah Hah');
legend('linear_vel','angular_vel')







