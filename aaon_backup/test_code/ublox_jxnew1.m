
%V1000 to txt
% Display surf plot of the currently selected data.
%mat 格式转换为
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%open and read
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

filePath=[PathName,'\\',FileName];
out_filePath=[filePath(1:end-4),'.mat'];
load(out_filePath);
m=size(UBX,1);
len_m=floor(m/32);
ID=zeros(len_m,255);
snr=UBX(:,15);
id=UBX(:,16);

for i=1:(len_m*32-1)
    %GPS
    i_32=floor(i/32)+1;
    if(id(i)~=0)
        ID(i_32,id(i))=snr(i);
    end
end
len=(size(ID,1));
t=(1:len)*0.1*32;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ID_is_mun=zeros(1,255);
%         ID_is    =zeros(1,255);
for j=1:255
    ID_is_mun(j)=sum((ID(:,j)~=0));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
del_num=0.9;%有效数据百分毿
id_plot=1:32;%绘图范围
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ID_is=ID_is_mun>del_num*len_m;
num_is=find(ID_is(id_plot)==1);
plot(UBX(1:32:32*len_m,2)'*1e-4,ID(:,id_plot(num_is)));



















