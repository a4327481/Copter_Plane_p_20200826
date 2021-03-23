% function accel_radg()   
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

% fileID = fopen([PathName,'\\',FileName]);
% C = fread(fileID);
% fclose(fileID);

fileID = fopen([PathName,'\\',FileName]);
Cin = textscan(fileID,'%x');
fclose(fileID);
C=Cin{1,1};

len=length(C);
BLOCK_SIZE=5;
m=floor(len/BLOCK_SIZE);
ax=zeros(m,1);
i=1;
j=1;
while(i<(m-1)*BLOCK_SIZE)
    if(C(i+4)==mod(sum(C(i:i+3)),256))
        ax(j)=double(typecast(uint8(C(i+3:-1:i)),'int32')');            
        j=j+1;          
        i=i+BLOCK_SIZE;
    else
        i=i+1;
    end
    
end
t=((1:j-1)*0.001)';
ax(j:end)=[];





