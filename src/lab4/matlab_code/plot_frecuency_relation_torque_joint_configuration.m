
clc, clear all, close all,

s= tf('s');

G1 = 10/(s+10);

w = linspace(1,7,1000);
bode(G1, w)

%% 
clc, clear all, close all
s = tf('s');

K = 500; D = 50;

G = K/(s*s + D*s + K);

s = tf('s');

K = 500; D = 50;

w = linspace(1,7,1000);
bode(G, w)