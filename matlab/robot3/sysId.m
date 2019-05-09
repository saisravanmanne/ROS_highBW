clc
close all
clear all 
l1 = 1;
l2 = 7051   ; %5376 %4197 % long 4727
Ts = 1/80; % in seconds
killerKb = csv2table('sysID_long3.csv',l1,l2);  % sysID_long.csv is bad data

output_A = table2array(killerKb(:,2));
output_B = table2array(killerKb(:,4));
input_A = table2array(killerKb(:,6));
input_B = table2array(killerKb(:,8));

output = [output_A output_B];
input = [input_A input_B]; 

data = iddata(output,input,Ts);
plot(data);

sysTF = tfest(data,4,3,0)

clf
compare(data,sysTF)


