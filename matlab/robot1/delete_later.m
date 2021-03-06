clc 
clear all 
s = tf([1 0],[1]);
z = tf('z');
ts = 1/105;
P1 = 1.65/(s + 1.67);

P2 = 3.310/(s + 7.14); % from PWM to Wr,l - use this for inner-loop design
P21 = 33.89/(s + 7.14); % from Volt to Wr,l
C2 = 7.07 + 19.4/s;
C3 = 22.9 + 99.5/s;
C4 = 2.65 + 26/s
C5 = 0.259 + 4.45/s
C2d = c2d(C2,ts,'zoh')
C3d = c2d(C3,ts,'zoh')
C4d = c2d(C4,ts,'zoh')
C5d = c2d(C5,ts,'zoh')

md = 0; % additional mass that has to be added 
d = 0; Veq = 2; Weq = 0.8; % in m/s max value is 0.14 for hardware 
L = 0.434; dw = 0.324; R = 0.039; % default values L = 0.3536 //\\ has to be chosen based on the corresponding AR value (AR_calculation.m)
Iw = 6.8394e-04; % default values //\\ has to be chosen based on the corresponding AR value
I = I_Newcalculation(d,Iw,L,md,dw);
%[max,min] = Imaxmin(d,Iw,L,md,dw);
Plant1 = Plantww(d, Veq, Weq, dw, Iw, I, L, md,R);
hold on;
step(Plant1(1,1))
step(P21)





%% This controller worked completely fine Tr < 0.3
%CL = CL_p + 1.8*Lerror - 1.727*Lerror_p;

%CR = CR_p + 1.8*Rerror - 1.727*Rerror_p;

%% this is the second controller agressive bust more robust # theBestCurve


%CL = CL_p + 7.07*Lerror - 6.885*Lerror_p;

%CR = CR_p + 7.07*Rerror - 6.885*Rerror_p;

%% this is the third controller max BW (40 rad/s) 
%the motors couldn't accomadate this BW, the rise time ~0.5, high tracking
%error
%CL = CL_p + 22.9*Lerror - 21.95*Lerror_p;

%CR = CR_p + 22.9*Rerror - 21.95*Rerror_p;

%% this is the fourth controller max BW (10 rad/s) 
% actual motor model 

%CL = CL_p + 2.65*Lerror - 2.402*Lerror_p;

%CR = CR_p + 2.65*Rerror - 2.402*Rerror_p;

%% this the 2 rad/sec BW controller 
%CL = CL_p + 0.259*Lerror - 0.2166*Lerror_p;

%CR = CR_p + 0.259*Rerror - 0.2166*Rerror_p;