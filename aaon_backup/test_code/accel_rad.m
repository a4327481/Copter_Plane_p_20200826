%  
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

fileID = fopen([PathName,'\\',FileName]);
C = fread(fileID);
fclose(fileID);
len=length(C);
BLOCK_SIZE=11;
m=floor(len/BLOCK_SIZE);
ax=zeros(m,1);
ay=zeros(m,1);
az=zeros(m,1);
i=1;
j=1;
while(i<m*BLOCK_SIZE)
    if(C(i)==hex2dec('AA')&&C(i+1)==hex2dec('AA')&&C(i+2)==hex2dec('F1')&&C(i+3)==hex2dec('06'))
        ax(j)=double(typecast(uint8([C(i+5),C(i+4)]),'int16')')*0.01;
        ay(j)=double(typecast(uint8([C(i+7),C(i+6)]),'int16')')*0.01;
        az(j)=double(typecast(uint8([C(i+9),C(i+8)]),'int16')')*0.01;
        j=j+1;
        i=i+11;
    else
        i=i+1;
    end
    
end
t=((1:j-1)*0.002)';
ax(j:end)=[];
ay(j:end)=[];
az(j:end)=[];
data_ck=[t ax ay az];
fid=fopen([PathName,'\\',FileName,'accel'],'w');
fprintf(fid,'t ax ay az\n');
fclose(fid);
save([PathName,'\\',FileName,'accel'],'data_ck','-ascii','-append' )
