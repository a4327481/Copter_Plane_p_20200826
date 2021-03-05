function accel_radg()   
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mat 格式转换为画图
global PathName
if PathName~=0
    cd(PathName);
    [FileName,PathName,~] = uigetfile([PathName,'\\*.*']); % 读取雷达数据
else
    [FileName,PathName,~] = uigetfile('*.*'); % 读取雷达数据
end
if FileName==0
    return;
end

plot_big=0;%%%%%%%%%% plot_big=1，绘制大图；plot_big=0，不绘制大图

fileID = fopen([PathName,'\\',FileName]);
C = fread(fileID);
fclose(fileID);
len=length(C);
BLOCK_SIZE=9;
m=floor(len/BLOCK_SIZE);
ax=zeros(m,1);
ay=zeros(m,1);
az=zeros(m,1);
i=1;
j=1;
while(i<(m-1)*BLOCK_SIZE)
    if(C(i)==hex2dec('AA')&&C(i+1)==hex2dec('FF'))
        ax(j)=double(typecast(uint8([C(i+2),C(i+3)]),'int16')')*0.01;
        ay(j)=double(typecast(uint8([C(i+4),C(i+5)]),'int16')')*0.01;
        az(j)=double(typecast(uint8([C(i+6),C(i+7)]),'int16')')*0.01;
        if((C(i+8)==mod(sum(C(i+2:i+7)),256)&&(abs(ax(j))<20)&&(abs(ay(j))<20)&&(abs(az(j))<20)))
            j=j+1;
            i=i+9;
        else
            i=i+1;
        end

    else
        i=i+1;
    end
    
end
t=((1:j-1)*0.001)';
ax(j:end)=[];
ay(j:end)=[];
az(j:end)=[];
data_ck=[t ax ay az];%数据数组
% if(plot_big==1)
% figure(1)
% plot(t,ax)
% legend('ax')
% figure(2)
% plot(t,ay)
% legend('ay')
% figure(3)
% plot(t,az)
% legend('az')
% end
% figure(4)
% subplot(3,1,1);
% plot(t,ax)
% legend('ax')
% subplot(3,1,2);
% plot(t,ay)
% legend('ay')
% subplot(3,1,3);
% plot(t,az)
% legend('az')
% end
fid=fopen([PathName,'\\',FileName,'accel'],'w');
fprintf(fid,'t ax ay az\n');
fclose(fid);
save([PathName,'\\',FileName,'accel'],'data_ck','-ascii','-append' )
