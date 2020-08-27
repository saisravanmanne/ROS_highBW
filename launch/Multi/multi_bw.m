clc 
close all;
clear all;

killerKb1 = csv2table('multi_BWd.csv',1,1604);
killerKb2 = csv2table('multi_BWk.csv',1,612);


x1 = table2array(killerKb1(:,2));
y1 = table2array(killerKb1(:,4));

x2 = table2array(killerKb2(:,2));
y2 = table2array(killerKb2(:,4));


x3 = table2array(killerKb1(:,14))
y3 = table2array(killerKb1(:,16))

figure; hold on ;
plot(x3,y3,'b');
plot(x1,y1,'--g');
plot(x2,y2,'--r');
grid on;
h_axes = findobj(gcf, 'type', 'axes');
xlabel('x (m)','FontSize',12);
ylabel('y (m)','FontSize',12);
set(h_axes,'LineWidth',2,'FontSize',12,'GridAlpha',0.15); % size and brightness of grid and size of x & y axis numbers
title('Trajectory: Reference vs Actual','FontWeight','bold','FontSize',14, 'Interpreter','latex')

h_line = findobj(gcf, 'type', 'line');
set(h_line, 'LineWidth',2);         % Lines with thicker width for plots


%% Xe and Ye plots 

v1 = table2array(killerKb1(:,2));
v2 = table2array(killerKb2(:,2));
vref1 = table2array(killerKb1(:,14));
vref2 = table2array(killerKb2(:,14));
vtime1 = table2array(killerKb1(:,12));
vtime2 = table2array(killerKb2(:,12));

theta1 = table2array(killerKb1(:,4));
theta2 = table2array(killerKb2(:,4));
tref1 = table2array(killerKb1(:,16));
tref2 = table2array(killerKb2(:,16));
ttime1 = table2array(killerKb1(:,12));
ttime2 = table2array(killerKb2(:,12));

ve1 = abs(vref1 - v1);
ve2 = abs(vref2 - v2);

thetae1 = abs(tref1 - theta1);
thetae2 = abs(tref2 - theta2);

figure;
plot(ttime1,ve1,ttime2,ve2);

figure;
plot(ttime1,thetae1,ttime2,thetae2);