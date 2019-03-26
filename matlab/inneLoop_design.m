%% Inner Loop Design and Plant Analysis 
clc 
close all 
clear all 
%% constant declerations
m = 3.02 % in Kg
Iz = 0.0421502 % in Kgm^2
r = 0.06 % in m
d_w = 0.33 % in m 




L = [0.28] ; % Length of robot 
W = [0.254] ; % width of robot base
d_w = [0.36] ; % width including wheels
Kb = [0.0019] ; % Back emf in Volts
Kt = [Kb] ;% Torque constant
Kg = [9.68] ; %9.68
Ra = [(4.641+3.934)/2] ;
La = [0.5*(1367.7+1389.9)*1e-6];
dcgain = 9.754 ; % DC gain
dominant_pole = 4.1667 ; % dominant pole 
Iw = (Kt/(Kg*Ra))/(dominant_pole*dcgain) ;
Im = Iw/2 ;
b = (dominant_pole*Iw*Ra - Kt*Kb)/Ra ;





c = 1;
%% Linearized Plant State Space Matrixes
% Plant from [er el] to [wr wl]
%
A = [-2*(b*Kg(c)*Kg(c))/(m*r*r)    0    (Kt*Kg(c))/(m*r)    (Kt*Kg(c))/(m*r) ;
      0    (-b*Kg*Kg*d_w*d_w)/(2*Iz*r*r)    (Kg*Kt*d_w)/(2*Iz*r)    (-Kg*Kt*d_w)/(2*Iz*r);
      -Kb*Kg(c)/(La*r)    -Kb*Kg(c)*d_w/(2*La*r)    -Ra/La    0 ;
      -Kb*Kg(c)/(La*r)    Kb*Kg(c)*d_w/(2*La*r)    0    -Ra/La ];
B = [0 0; 0 0; 1/La 0; 0 1/La];
C = [1/r  d_w/(2*r) 0 0 ;
     1/r -d_w/(2*r) 0 0];
D = [0 0; 0 0];

P_ss = ss(A,B,C,D)
s = tf([1 0],[1]);
P_tf = tf(P_ss);
P_tf = zpk(P_tf)
P_approx = [P_tf(1,1) 0; 0 P_tf(2,2)]
%Finding the transmission zeros involved for the low freq approximation
%model
P_tf = P_approx;
z = tzero(P_tf);
H1 = evalfr(P_tf,z(1));
svd(H1);
H1 = evalfr(P_tf,z(2));
svd(H1);
H1 = evalfr(P_tf,z(3));
svd(H1);
H1 = evalfr(P_tf,z(4));
svd(H1);
% for the low freq approximation model there are no transmission zeros 
%% PI Controller Design 

% plant model based on the empherical data
BW = 2*pi*20;
ta = 1/1260;
ph = 60*pi/180;
DC_value = 38;
rise_time = 3;
domi_pole = 5/(rise_time);
P = (DC_value*domi_pole)/(s+domi_pole)
pre = 100/(s+100);
[Mag, Pha, BW] = bode(P,BW); 
% PI = g(s+z)/s
z = BW/(tan(ph - (pi/2) - (Pha*pi/180)));
g = BW/((sqrt((z^2)+(BW^2)))*(Mag));
PI = g*(s+z)/s
CL = PI*P/(1+PI*P);
bode(CL*pre);
g;
g*z;
c2d(100/(s+100),0.01,'tustin');

z = BW/(tan((ph - (pi/2) - (Pha*pi/180) + atan(ta*BW))/2));
k = BW*sqrt(((ta*BW)^2)+1)/(((z^2)+(BW^2))*Mag);
PID = k*((s+z)^2)/(s*(ta*s+1));
bode(PID*P)
CL2 = PID*P/(1+PID*P);
allmargin(CL2)
k
z

g = k;
ta = 1/1260;
Po = 1;
A = ((2*g*z) - (ta*g*z*z))/ta ;
B = g*z*z*td ;
C = (g*(z*ta - 1)*(z*ta - 1))/ta ;
Po = 1 ;

CL = CL_p*(3 + Po) + CL_pp*(-3 -3*Po) + CL_ppp*(1 + 3*Po) + CL_pppp*(-Po) - (Lerror_p*(A+C) + Lerror_p*(-A*Po -A + B - 2*C) + Lerror_p*(A*Po - B*Po + C));  
CR = CR_p*(3 + Po) + CR_pp*(-3 -3*Po) + CR_ppp*(1 + 3*Po) + CR_pppp*(-Po) - (Rerror_p*(A+C) + Rerror_p*(-A*Po -A + B - 2*C) + Rerror_p*(A*Po - B*Po + C));  
% PI 6


