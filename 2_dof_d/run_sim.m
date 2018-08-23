clc
clear
close all


tf = 10;
t = linspace(0,tf,10000);
phi = 15*sin(115/15*t)*pi/180;
theta = 20*sin(10*t)*pi/180;

phi = cat(2,t',phi');
theta = cat(2,t',theta');
sim('gimbal.slx')