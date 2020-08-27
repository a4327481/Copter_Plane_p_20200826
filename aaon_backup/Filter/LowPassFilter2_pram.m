%lowpassfilter2 test
Tn=0.001;%s
% sample_freq=1/Tn/2/pi;
% curr_freq=10;%hz

sample_freq=1/Tn;
curr_freq=2;%hz
T=1/curr_freq/2/pi;
wn=curr_freq*pi*2;
zuni=cos(pi/4.0);
