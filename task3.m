%% task 3: generate frequency ramp
clear all
close all
fs = 48000;
fstart = 200;
fstop = 1800;
t = 0.1; %s

steps = t * 48000;
slope = (fstop-fstart)/fs/steps;
phi = 0;
for n = 2:steps
    phi(n) = (2*pi*(fstart*n + slope*n^2/2));
    phi(n) = mod(2*pi*(fstart*n + slope*n^2/2),2*pi);
    sine(n) = cos(phi(n));
end
plot(phi)
figure
plot(sine);
figure
stft(sine,fs);