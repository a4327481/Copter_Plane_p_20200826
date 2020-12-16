%% 导入文本文件中的数据。
% 用于从以下文本文件导入数据的脚本:
%
%    E:\飞行试验\V10\20200907\第2架次-手动切换\第2架次-手动切换\LOG00027.TXT
%
% 要将代码扩展到其他选定数据或其他文本文件，请生成函数来代替脚本。

% 由 MATLAB 自动生成于 2020/09/08 18:20:11

%% 初始化变量。
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
filename=([PathName,'\\',FileName]);
data=importdata([PathName,'\\',FileName]);
for i=1:5
    a=isnan(data(:,i));
    data(a,:)=[];
    b=data(:,i)>1000;
    data(b,:)=[];
end
len=max(size(data));

% data(:,1)=(data(1,1)-data(:,1))/2;
a_sum=data(:,1)+data(:,2)+data(:,3)+data(:,4)+data(:,5);
data1=[(1:len)'/40 data(:,1:5) a_sum data(:,6:12)];

head=['t Lu Ld Ru Rd tail a_sum L_a_A L_e_A r_A L_a_T reser L_e_T a5_rpm','\n'];
data_ck=data1;
fid=fopen([filename,'V1000.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
    save([filename,'V1000.dat'],'data_ck','-ascii','-append' )
