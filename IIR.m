theta = 800/48000;

n = 1:10000;

ystep = (1-cos(theta*2*pi*(n))) / (theta*2*pi*sin(theta*2*pi));


offset = - max(ystep)/2;
scaling_f = (2^15-1)/max(ystep)*2;

plot(scaling_f*(offset + ystep));