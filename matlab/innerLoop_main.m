clc
close all
clear all
loop = 3; % 1 - PID design %2 Wr,Wl design %3 ICC Paper Plant 
s = tf([1 0],[1]);
%% Plant Model 
m = 2.96; mc = 2.96 - 0.279; I = 0.0285 ; Iw =   8.0306e-05; L = 0.28/2; R = 0.0610; d = 0.0; dw = 2*L; 
Weq = 3.14/2; Veq = 1.2;
Ao = m + 2*Iw/(R^2); Bo = I + (2*Iw*L^2)/R^2;
% Linear Plant without actuator - Motor Torque to V,W 
A = [0 2*mc*d*Weq/Ao; -mc*d*Weq/Bo -mc*d*Veq/Bo] ; B = [1/(R*Ao) 1/(R*Ao); L/(R*Bo) -L/(R*Bo)]; M = [1 0; 0 1]; D = [0 0;0 0];
mainsys = ss(A,B,M,D);
Tmain = tf(mainsys);

% Linear Plant without actuator - Motor Torque to Wr, Wl
Tmain2 = [(1/(s*R*R))*((1/Ao)+(L*L/Bo)) (1/(s*R*R))*((1/Ao)-(L*L/Bo)); (1/(s*R*R))*((1/Ao)-(L*L/Bo)) (1/(s*R*R))*((1/Ao)+(L*L/Bo))];

% Right Motor Actuator Dynamics - voltage to motor torque 
Kt = 0.0337; Kg = 9.68; Kb = Kt; B = 1.3023e-04; La = 22.8e-06; Ra = 2.9;
s = tf([1 0],[1]);
TactR = Kt*Kg*(Ra/(La*s + Ra))*(s*Iw + B)/((s*Iw + B)*(Ra) + Kb*Kt); TactR = zpk(TactR);

% Left Motor Actuator Dynamics - voltage to motor torque 
Kt = 0.0249; Kg = 9.68; Kb = Kt; B = 1.4537e-05; La = 23.62e-06; Ra = 3.3;
s = tf([1 0],[1]);
TactL = Kt*Kg*(Ra/(La*s + Ra))*(s*Iw + B)/((s*Iw + B)*(Ra) + Kb*Kt); TactL = zpk(TactL);

% Plant including Actuator Dyanmics input voltage to V, W
Plant = minreal(Tmain*[TactR 0; 0 TactL]); Plant = [Plant(1,1) 0; 0 Plant(2,1)];

% Plant including Actuator Dyanmics input voltage to Wr, Wl
Plant2 = minreal(Tmain2*[TactR 0; 0 TactL])

Plantss = ss(Plant,'minimal');

% Alternate representation of Plant using State Space form ACC paper 
Kg = 1; Iw = 0;

As = [(-2*B*Kg^2)/((m*R^2)+Iw) 0 Kt*Kg/(La*R*Ao) Kt*Kg/(La*R*Ao)
       0   -2*L*L*Kg*Kg*B/(2*R*R*Bo) Kt*Kg*L/(La*R*Bo) -Kt*Kg*L/(La*R*Bo)
       -Kb*Kg/R -Kb*Kg*L/R -Ra/La 0
       -Kb*Kg/R Kb*Kg*L/R 0 -Ra/La];
Bs = [0 0; 0 0; 1 0; 0 1]; Cs = [1/R L/R 0 0; 1/R -L/R 0 0]; Ds = [0 0;0 0];
Mains = ss(As,Bs,Cs,Ds);
MainTfs = minreal(tf(Mains)); Mains = ss(MainTfs); MainTfs = minreal(zpk(MainTfs));

% Alternate representation of Plant from Lin's thesis ea -r,l to Wr,Wl 
H1 = Kt/(La*m*R*R*s*s + (Ra*m*R*R + 2*La*B)*s + (2*Kb*Kt + 2*Ra*B));
H2 = dw*dw*Kt/(I*La*R*R*s*s + (I*Ra*R*R + dw*dw*La*B)*s + (Kb*Kt*L*L + dw*dw*Ra*B));
Plant3 = [H1+0.5*H2  H1-0.5*H2; H1-0.5*H2 H1+0.5*H2]; Plant3 = minreal(zpk(Plant3));

% Alternate representation of Plant from CCTA x = [v w Iar Ial]
A = [(-2*B*Kg^2)/((m*R^2)) 0  Kt*Kg/(La*R*m*R) Kt*Kg/(La*R*m*R) 
      0 -B*Kg*Kg*dw*dw/(2*I*R*R) Kg*Kt*dw/(2*I*R) -Kg*Kt*dw/(2*I*R)
      -Kb*Kg/(La*R) -Kb*Kg*dw/(2*La*R) -Ra/La 0
      -Kb*Kg/(La*R) Kb*Kg*dw/(2*La*R) 0 -Ra/La];
B = [0 0; 0 0; 1/La 0; 0 1/La]; C = [1/R L/R 0 0; 1/R -L/R 0 0]; D = [0 0; 0 0];
Plant4 = ss(As,Bs,Cs,Ds);
Plant4 = minreal(tf(Plant4)); Plant4 = ss(Plant4); Plant4 = minreal(zpk(Plant4));

if(loop == 1)

Kp = 94.8; Ki = 59.3; Kd = 29.2; Tf = 0.0961; Ts = 1/105;
P1 = Plant(1,1); rlocus(P1); K = Kp + Ki/s; K11 = K;
Pcl = minreal((K*P1)/(1+K*P1)); step(Pcl)
Pk = minreal((K)/(1+K*P1)); step(Pk)
Ts = 1/105; Kd = c2d(C,Ts,'zoh') %%  y = yp + kpu - kpup + tskiup % Kp = 29.5; Ki = 139; Ts = 0.00952
Ts = 1/105; Pcld = c2d(Pcl,Ts,'zoh'); step(Pcld);
Ts = 1/105; Pkd = c2d(Pk,Ts,'zoh'); step(Pkd)

 % PID = (Kp*Tf + Kd)/Tf  + (-2*Kp*Tf -2*Kd + Kp*Ts + Ki*Ts*Tf)/Tf + (Kp*Tf + Kd - Kp*Ts -Ki*Ts*Tf + Ki*Ts*Ts)/Tf -(-2*Tf + Ts)/Tf - (Tf - Ts)/Tf
Kp = 89.3; Ki = 57; Kd = 23.7; Tf = 0.102; Ts = 1/105;
P2 = Plant(2,2); rlocus(P2); K = Kp + Ki/s; K22 = K;
Pcl = minreal((K*P2)/(1+K*P2)); step(Pcl);
Pk = minreal((K)/(1+K*P2)); step(0.8*Pk)
Ts = 1/105; Cd = c2d(C,Ts,'zoh') %%y = yp + kpu - kpup + tskiup % Kp = 2.46; Ki = 19.8; Ts = 0.00952
Ts = 1/105; Pcld = c2d(Pcl,Ts,'zoh'); step(Pcld);
Ts = 1/105; Pkd = c2d(Pk,Ts,'zoh'); step(Pkd)

K = [K11, 0 ; 0 K22]; step(K)
t = [0 : 1/105 : 8]; lsim([0.5 0.5; 0.5 -0.5]*[(K11)/(1+K11*P1) 0; 0  (K22)/(1+K22*P2)],[1.12*ones(size(t)); 0.8*ones(size(t))],t)

end 

if (loop == 2)

    P = sysTF; 
    D1 = -(P(1,2)/P(1,1)); D2 = -(P(2,1)/P(2,2)); D = [1 D1; D2 1];
    Ts = 1/105; D1d = c2d(D1,Ts,'zoh');  D2d = c2d(D2,Ts,'zoh'); 
    PP = P*D; 
    
    Kp = 97.6; Ki = 62.3; Kd = 28.7; Tf = 0.0961; Ts = 1/105; Ao = -4.5467e-05; A1 = - 1.984; A2 = 0.9844; B1 = - 2; B2 = 1;
    P1 = P(1,1);
    %PID = (Kp*Tf + Kd)/Tf  + (-2*Kp*Tf -2*Kd + Kp*Ts + Ki*Ts*Tf)/Tf +
    %(Kp*Tf + Kd - Kp*Ts -Ki*Ts*Tf + Ki*Ts*Ts)/Tf -(-2*Tf + Ts)/Tf - (Tf - Ts)/Tf
    
    %y = Ao + Ao*A1 + Ao*A2 - B1 - B2
    Kp = 25.4; Ki = 35; Kd = 29.74; Tf = 0.03; Ts = 1/105; Ao =  0.059638; A1 = - 1.982; A2 = 0.9818; B1 = - 1.999; B2 = 0.9988;
    P2 = P(2,2);
end

if (loop == 3)
     P = Plant4;  plantzeros = tzero(Plant4); 
     Kp = 20; Ki = 0.05; Kd = 17;
     K = Kp + (Ki/s) + Kd*s;
     Pcl = K*P/(1+K*P); step(Pcl)
     
     step(2.03*(s + 0.04751)/((s + 0.07006)*(s + 0.03627)))
end 
