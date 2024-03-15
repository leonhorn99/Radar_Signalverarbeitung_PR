%% day 2 overlap and add
clear all
close all
%filter parameters
order = 63;
blockSize = 128;
window = hamming(order +1);
fs = 48000;

filterp = fir1(order,[1000/fs, 8000/fs],"bandpass",window);
steps = 10;

%step test signal
stepS = rand([blockSize * steps,1]);
realConv = conv(stepS,filterp);
overlapAddConv=zeros([blockSize*steps+order,1]);

for s = 1:steps
    overlapAddConv((s-1)*blockSize+1:(s)*blockSize+order) = overlapAddConv((s-1)*blockSize+1:(s)*blockSize+order) + sum((stepS((s-1)*blockSize+1:s*blockSize),filterp);
end

plot(realConv);
figure
plot(realConv);
hold on;
plot(overlapAddConv);

%% C like implementation

x = ones([1000,1]);
