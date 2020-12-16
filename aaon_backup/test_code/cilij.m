% V100_to_mat_load_ins


Magbx=MagM0;
Magby=MagM2;
Magbz=-MagM1;

Magx=MagM2X;
Magy=MagM2Z;
Magz=-MagM2Y;

len=length(pitch);
Mag=zeros(len,3);
for i=1:len
 
Cnb = a2mat([pitch(i) roll(i) 0]');
Mag(i,:)=(Cnb*[Magby(i) Magbx(i) -Magbz(i)]')';   
end
MagE=Mag(:,1);
MagN=Mag(:,2);
MagU=Mag(:,3);
yawM=atan2d(MagE,MagN);
figure
plot(time,MagM0,time,MagM1,time,MagM2,time,MagABS)
legend('MagM0','MagM1','MagM2','MagABS')
figure
plot(time,MagM2X,time,MagM2Y,time,MagM2Z,time,MagABS2)
legend('MagM2X','MagM2Y','MagM2Z','MagABS2')
figure
plot(time,Mag(:,1),time,Mag(:,2),time,Mag(:,3))
legend('MagE','MagN','MagMU')
figure
plot(time,PSI,time,yawM)
% plot(time,MagM0,time,MagM2X)
% 
% plot(time,MagM1,time,MagM2Y)
% 
% plot(time,MagM2,time,MagM2Z)
% 
% plot(time,pitch*HD,time,roll*HD)