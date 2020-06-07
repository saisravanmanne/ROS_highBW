clc 
clear all 
s = tf([1 0],[1]);
z = tf('z');
ts = 1/105;
P1 = 1.65/(s + 1.67);
C2 = 7.07 + 19.4/s;
C3 = 22.9 + 99.5/s;
C2d = c2d(C2,ts,'zoh')
C3d = c2d(C3,ts,'zoh')








%% This controller worked completely fine Tr < 0.3
%CL = CL_p + 1.8*Lerror - 1.727*Lerror_p;

%CR = CR_p + 1.8*Rerror - 1.727*Rerror_p;

%% this is the second controller agressive bust more robust # theBestCurve


%CL = CL_p + 7.07*Lerror - 6.885*Lerror_p;

%CR = CR_p + 7.07*Rerror - 6.885*Rerror_p;

%% this is the third controller max BW (40 rad/s) 
%the motors couldn't accomadate this BW, the rise time ~0.5, high tracking
error
%CL = CL_p + 22.9*Lerror - 21.95*Lerror_p;

%CR = CR_p + 22.9*Rerror - 21.95*Rerror_p;