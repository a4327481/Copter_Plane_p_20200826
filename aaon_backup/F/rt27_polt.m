% function  mat_to_matlab(varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%mat 格式转换为画图
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

%% 初始化变量。
filename = [PathName,'\\',FileName];
startRow = 2;

%% 每个文本行的格式:
%   列1: 双精度值 (%f)
%	列2: 双精度值 (%f)
%   列3: 双精度值 (%f)
%	列4: 双精度值 (%f)
%   列5: 分类 (%C)
%	列6: 文本 (%s)
% 有关详细信息，请参阅 TEXTSCAN 文档。
formatSpec = '%14f%14f%9f%2f%11C%s%[^\n\r]';

%% 打开文本文件。
fileID = fopen(filename,'r');

%% 根据格式读取数据列。
% 该调用基于生成此代码所用的文件的结构。如果其他文件出现错误，请尝试通过导入工具重新生成代码。
dataArray = textscan(fileID, formatSpec, 'Delimiter', '', 'WhiteSpace', '', 'TextType', 'string', 'EmptyValue', NaN, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% 删除围绕所有元胞列的空白。
dataArray{6} = strtrim(dataArray{6});

%% 关闭文本文件。
fclose(fileID);

%% 对无法导入的数据进行的后处理。
% 在导入过程中未应用无法导入的数据的规则，因此不包括后处理代码。要生成适用于无法导入的数据的代码，请在文件中选择无法导入的元胞，然后重新生成脚本。

%% 创建输出变量
GPS_data = table(dataArray{1:end-1}, 'VariableNames', {'LONGITUDE','LATITUDE','HEIGH','Quality','year_month_day','hour_min_sec'});
len=height(GPS_data);
ts.sec=zeros(len,1);
ts.min=zeros(len,1);
ts.hour=zeros(len,1);
ts.hm_sec=zeros(len,1);
for i=1:len
%     ts.hour(i)=str2num(GPS_data.hour_min_sec{i}(1:2));
%     ts.min(i) =str2num(GPS_data.hour_min_sec{i}(4:5));
%     ts.sec(i)=str2num(GPS_data.hour_min_sec{i}(7:end));
%     ts.hm_sec(i)=ts.hour(i)*60*60+ts.min(i)*60+ts.sec(i);
%     ts.hm_sec(i)=str2num(GPS_data.hour_min_sec{i}(1:2))*60*60+str2num(GPS_data.hour_min_sec{i}(4:5))*60+str2num(GPS_data.hour_min_sec{i}(7:end));
    ts.hm_sec(i)=str2num(dataArray{1, 6}{i}(1:2))*60*60+str2num(dataArray{1, 6}{i}(4:5))*60+str2num(dataArray{1, 6}{i}(7:end));
%     dataArray{1, 6}
end
Lat=GPS_data{:,2};
Lng=GPS_data{:,1};
alt=GPS_data{:,3};

Re=6378137;
Xp=Re*Lng/57.3.*cosd(Lat(1));
Yp=Re*Lat/57.3;
Xp=Xp-Xp(1);
Yp=Yp-Yp(1);
V=((Xp(2:end)-Xp(1:end-1)).^2+(Yp(2:end)-Yp(1:end-1)).^2).^0.5./(0.02);
Vz=(alt(2:end)-alt(1:end-1))./(0.02);
% V=((Xp(2:end)-Xp(1:end-1)).^2+(Yp(2:end)-Yp(1:end-1)).^2).^0.5./(ts.hm_sec(2:end)-ts.hm_sec(1:end-1));
% Vz=(alt(2:end)-alt(1:end-1))./(ts.hm_sec(2:end)-ts.hm_sec(1:end-1));
V=[0 ;V];
Vz=[0 ;Vz];


data_ck=[ts.hm_sec-ts.hm_sec(1) GPS_data{:,1:3} Xp Yp V Vz];
fid=fopen([PathName,'\\',FileName,'RT27_DATA.dat'],'w');
fprintf(fid,'ts LONGITUDE LATITUDE HEIGH Xp Yp V Vz\n');
    [count,~]=size(data_ck);
     for i=1:count
        fprintf(fid,'> %f %f %f %f %f %f %f %f\n',data_ck(i,:));
    end
    fclose(fid);

%% 清除临时变量
% clearvars filename startRow formatSpec fileID dataArray ans;