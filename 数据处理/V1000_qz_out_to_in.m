HD=180/pi;
L0=35;
L1=23;
L2=25.93;
syms Qo Qi
eqn=-L0^2+(L0+L2*cos(Qo)-L1*cos(Qi))^2+(L2*sin(Qo)-L1*sin(Qi))^2 == 0;
S=solve(eqn,Qi);

out1=(-96:-31)';
out2=(-30:7)';
outx1=(165-95.56-out1);
outx2=(165-95.56-out2);

a=subs(S(1),Qo,outx1/HD);
in1=real(double((a)))*HD;
in1=(in1<0)*360+in1;
b=subs(S(1),Qo,outx2/HD);
in2=real(double((b)))*HD;
in=[in1;in2];
out=[out1 ;out2];

     data_qz={
        'out';
        'in';
        };
data_3 = [];
filename='E:\matlab_code\Copter_Plane\V1000';
for i=1:length(data_qz)
        data_3(:,i)=eval(data_qz{i,1}); 
        end
        head=[data_qz{1}];
        for i=2:length(data_qz)
         head=[head ,' ',data_qz{i}];
        end 
        head=[head ,'\n'];
data_ck=data_3(:,1:i);
fid=fopen([filename,'\\','V1000_qzo.dat'],'w');
fprintf(fid,head);
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%.2f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid); 
% save([filename,'\\','V1000_qz.dat'],'data_ck','-ascii','-append' )
plot(in,out)