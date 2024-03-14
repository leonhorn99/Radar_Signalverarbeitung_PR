N = 1000;
f0 = 2000;
fs = 48000;

dPhi = 2*pi*(f0/fs);
re_d_phi = cos(dPhi);
im_d_phi = sin(dPhi);

re_sine = 0;
im_sine = 1;

for n = 2:N
    re_sine(n) = re_sine(n-1)*re_d_phi - im_sine(n-1)*im_d_phi;
    im_sine(n) = im_sine(n-1)*re_d_phi + re_sine(n-1)*im_d_phi;
end

plot(re_sine*(2^15-1));
hold on;
plot(im_sine*(2^15-1));