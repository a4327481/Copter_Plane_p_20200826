%c2d 
F_Hz=35;%截止频率 单位Hz
% imu_filt=1/(1/0.6341*0.004-0.004)/2/pi;
imu_filt=35;
zuni=0.707;
wn=2*pi*F_Hz;
Tn=0.004;
tfs=tf([wn^2],[1 2*wn*zuni wn^2]);
tfs1=tf([wn],[1 wn]);
tf_out=tfs;
%[wn],[1  wn]
% ftz=c2d(tf_out,Tn)
ftz=c2d(tf_out,Tn,'tustin')
% ftz.num{1:end}
% ftz.den{1:end}
