function [wr,wl] = invKin(v,omega)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
dw = 0.324;
R = 0.039;
wr = (v + omega*0.5*dw)/R;
wl = (v - omega*0.5*dw)/R;
end

