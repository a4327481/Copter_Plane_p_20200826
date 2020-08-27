% V100_to_mat_load_ins

Magbx=mag2_xx;
Magby=mag2_yx;
Magbz=mag2_zx;
MagABS=(Magbx.^2+Magby.^2+Magbz.^2).^0.5;

pitch_mag=interp1(time_algo,algo_pitch,time_mag);
roll_mag=interp1(time_algo,algo_roll,time_mag);

len=length(time_mag);
Mag=zeros(len,3);
for i=1:len
Cnb = a2mat([pitch_mag(i)/HD roll_mag(i)/HD 0]');
Mag(i,:)=(Cnb*[Magby(i) Magbx(i) -Magbz(i)]')';   
end
MagE=Mag(:,1);
MagN=Mag(:,2);
MagU=Mag(:,3);
yawM=atan2d(MagE,MagN);
figure
plot(time_mag,Magbx,time_mag,Magby,time_mag,Magbz,time_mag,MagABS)
legend('MagM0','MagM1','MagM2','MagABS')
figure
plot(time_mag,Mag(:,1),time_mag,Mag(:,2),time_mag,Mag(:,3))
legend('MagE','MagN','MagMU')
figure
plot(time_algo,algo_yaw,time_mag,-yawM,'*',time_ublox,ublox_cod)
% plot(time,MagM0,time,MagM2X)
% 
% plot(time,MagM1,time,MagM2Y)
% 
% plot(time,MagM2,time,MagM2Z)
% 
% plot(time,pitch*HD,time,roll*HD)