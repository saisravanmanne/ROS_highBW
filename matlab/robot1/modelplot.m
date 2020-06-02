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

velocity = table2array(killerKb(:,2));
RPM = table2array(killerKb(:,4));
ref_velocity = table2array(killerKb(:,6));
servo = table2array(killerKb(:,8));
position_x = table2array(killerKb(:,10));
position_y = table2array(killerKb(:,12));

%% plot the step response
figure;
plot(servo,RPM,'r');
xlabel({'servo (microsec)'})
ylabel({'RPM '})
title('MotorShit')
figure;
hold off;
grid on;
title('Robot3 Path Characteristics');
plot(position_x, position_y,'b');
xlabel({'X coordinate (meter)'});
ylabel({'Y coordinate (meter)'});







