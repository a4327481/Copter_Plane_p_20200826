HD=180/pi;
L0=35;
L1=23;
L2=25.93;
syms Qo Qi
eqn=-L0^2+(L0+L2*cos(Qo)-L1*cos(Qi))^2+(L2*sin(Qo)-L1*sin(Qi))^2 == 0;
S=solve(eqn,Qo);

in1=[55:1:180]';
a=subs(S(2),Qi,in1/HD);
out1=165-real(double((a)))*HD;
in2=[190:1:212]';
b=subs(S(1),Qi,in2/HD);
out2=165-real(double((b)))*HD;
in=[in1;in2];
out=[out1 ;out2]-95.56;

     data_qz={
        'in';
        'out';
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
fid=fopen([filename,'\\','V1000_qz.dat'],'w');
fprintf(fid,head);
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%.2f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid); 
    plot(in,out)
% save([filename,'\\','V1000_qz.dat'],'data_ck','-ascii','-append' )
