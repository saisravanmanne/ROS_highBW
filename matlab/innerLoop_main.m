clc
close all
clear all 
%% Plant Model 
m = 2.96; mc = 2.96 - 0.279; I = 0.0934 ; Iw =   -8.5978e-05; L = 0.28; R = 0.0610; d = 0.0;
Weq = 3.14/2; Veq = 1.2;
Ao = m + 2*Iw/(R^2); Bo = I + (2*Iw*L^2)/R^2;
% Linear Plant without actuator - Motor Torque to V,W 
A = [0 2*mc*d*Weq/Ao; -mc*d*Weq/Bo -mc*d*Veq/Bo] ; B = [1/(R*Ao) 1/(R*Ao); L/(R*Bo) -L/(R*Bo)]; M = [1 0; 0 1]; D = [0 0;0 0];
mainsys = ss(A,B,M,D);
Tmain = tf(mainsys);
% Right Motor Actuator Dynamics - voltage to motor torque 
Kt = 0.0337; Kg = 9.68; Kb = Kt; B = -1.3023e-04; La = 22.8e-06; Ra = 2.9;
s = tf([1 0],[1]);
TactR = Kt*Kg*(s*Iw + B)/((s*Iw + B)*(s*La + Ra) + Kb*Kt); TactR = zpk(TactR);

% Left Motor Actuator Dynamics - voltage to motor torque 
Kt = 0.0249; Kg = 9.68; Kb = Kt; B = -1.4537e-05; La = 23.62e-06; Ra = 3.3;
s = tf([1 0],[1]);
TactL = Kt*Kg*(s*Iw + B)/((s*Iw + B)*(s*La + Ra) + Kb*Kt); TactL = zpk(TactL);

% Plant including Actuator Dyanmics input voltage to V, W
Plant = Tmain*[TactR 0; 0 TactL]; Plant = zpk(Plant)
Plantss = ss(Plant,'minimal');

% Alternate representation of Plant using State Space form 

As = [(-2*B*Kg^2)/((m*R^2)+Iw) 0 Kt*Kg/(La*R*Ao) Kt*Kg/(La*R*Ao)
       0   -2*L*L*Kg*Kg*B/(2*R*R*Bo) Kt*Kg*L/(La*R*Bo) -Kt*Kg*L/(La*R*Bo)
       -Kb*Kg/R -Kb*Kg*L/R -Ra/La 0
       -Kb*Kg/R Kb*Kg*L/R 0 -Ra/La];
Bs = [0 0; 0 0; 1 0; 0 1]; Cs = [1 0 0 0; 0 1 0 0]; Ds = [0 0;0 0];
Mains = ss(As,Bs,Cs,Ds);
MainTfs = minreal(tf(Mains)); Mains = ss(MainTfs); MainTfs = zpk(MainTfs);


