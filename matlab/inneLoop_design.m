%% Inner Loop Design and Plant Analysis 
clc 
close all 
clear all 
%% constant declerations
m = 3.02 % in Kg
Iz = 0.0421502 % in Kgm^2
r = 0.06 % in m
d_w = 0.33 % in m 
s = tf([1 0],[1])

% plant model based on the empherical data
BW = 2*pi*80;
ta = 1/1260;
ph = 60*pi/180;
DC_value = 50;
rise_time = 6;
domi_pole = 5/(rise_time);
P = (DC_value*domi_pole)/(s+domi_pole)
pre = (s+10000)*(s+10000)/((s+669.2)*(s+669.2));
[Mag, Pha, BW] = bode(P,BW); 


z = BW/(tan((ph - (pi/2) - (Pha*pi/180) + atan(ta*BW))/2));
k = BW*sqrt(((ta*BW)^2)+1)/(((z^2)+(BW^2))*Mag);
PID = k*((s+z)^2)/(s*(ta*s+1));
bode(PID*P);
close all
CL2 = PID*P/(1+PID*P);
CLu = CL2/P ;
step(16*CLu)
allmargin(CL2);
k
z

g = k;
ta = 1/1260;
td = 0.01;
Po = exp(-td/ta);
A = ((2*g*z) - (ta*g*z*z)) ;
B = g*z*z;
C = (g*(z*ta - 1)*(z*ta - 1))/ta ;


% PI 6


