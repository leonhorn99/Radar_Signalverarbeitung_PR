%% day 2 overlap and add
clear all
close all
%filter parameters
order = 63;
blockSize = 128;
window = hamming(order +1);
fs = 48000;

filterp = fir1(order,1/4);
steps = 10;

%step test signal
stepS = ones([blockSize * steps,1]);
realConv = conv(stepS,filterp);
overlapAddConv=zeros([blockSize*steps+order,1]);


% plot(realConv);
% figure
% plot(realConv);
% hold on;
% plot(overlapAddConv);

%% C like implementation

x = (ones([1000,1]));
out = zeros([3000,1]);
old_overlap = zeros([order,1]);
i=1;
out_buf = zeros([blockSize,1]);
out_buf_ol = zeros([blockSize,1]);
while(1)
    x_buf = x((i-1)*blockSize+1:i*blockSize);
    y = zeros([2*blockSize,1]);
    for n = 1:blockSize
        for m = 1:(order+1)
            index = n+m-1;
            y(index) = y(index) + filterp(m)*x_buf(index-m+1);
        end
    end
    out_buf = y(1:blockSize) + out_buf_ol;
    out_buf_ol = y(blockSize+1:end);
    out((i-1)*blockSize+1:i*blockSize) = out_buf;
    i = i+1;
    if (i) * blockSize > length(x)
        break;
    end
end

figure
plot(y);
figure;
plot(out);
