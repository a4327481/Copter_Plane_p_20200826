%read_arspeeed
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
BLOCK_SIZE=9;
m=floor(len/BLOCK_SIZE)-1;
ax=zeros(m,1);
ay=zeros(m,1);
az=zeros(m,1);
i=1;
j=1;
while(i<m*BLOCK_SIZE)
    if(C(i)==hex2dec('AA')&&C(i+1)==hex2dec('FF'))
        ax(j)=double(typecast(uint8(C(i+2:i+3)),'int16')')/10;
        ay(j)=double(typecast(uint8(C(i+4:i+5)),'int16')')/10;
        az(j)=double(typecast(uint8([C(i+6),C(i+7)]),'int16')')/10;      
        if(C(i+BLOCK_SIZE-1)==mod(sum(C(i+2:i+BLOCK_SIZE-2)),256)&&abs(ax(j))<200&&abs(ay(j))<200&&abs(az(j))<200)
            j=j+1;
            i=i+BLOCK_SIZE;
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
data_ck=[t ax ay az];
fid=fopen([PathName,'\\',FileName,'accel'],'w');
fprintf(fid,'t ax ay az\n');
fclose(fid);
save([PathName,'\\',FileName,'accel'],'data_ck','-ascii','-append' )
