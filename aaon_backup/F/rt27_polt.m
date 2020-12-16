% function  mat_to_matlab(varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%mat ��ʽת��Ϊ��ͼ
global PathName
if PathName~=0
    cd(PathName);
    [FileName,PathName,~] = uigetfile([PathName,'\\*.*']); % ��ȡ�״�����
else
    [FileName,PathName,~] = uigetfile('*.*'); % ��ȡ�״�����
end
if FileName==0
    return;
end

%% ��ʼ��������
filename = [PathName,'\\',FileName];
startRow = 2;

%% ÿ���ı��еĸ�ʽ:
%   ��1: ˫����ֵ (%f)
%	��2: ˫����ֵ (%f)
%   ��3: ˫����ֵ (%f)
%	��4: ˫����ֵ (%f)
%   ��5: ���� (%C)
%	��6: �ı� (%s)
% �й���ϸ��Ϣ������� TEXTSCAN �ĵ���
formatSpec = '%14f%14f%9f%2f%11C%s%[^\n\r]';

%% ���ı��ļ���
fileID = fopen(filename,'r');

%% ���ݸ�ʽ��ȡ�����С�
% �õ��û������ɴ˴������õ��ļ��Ľṹ����������ļ����ִ����볢��ͨ�����빤���������ɴ��롣
dataArray = textscan(fileID, formatSpec, 'Delimiter', '', 'WhiteSpace', '', 'TextType', 'string', 'EmptyValue', NaN, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% ɾ��Χ������Ԫ���еĿհס�
dataArray{6} = strtrim(dataArray{6});

%% �ر��ı��ļ���
fclose(fileID);

%% ���޷���������ݽ��еĺ���
% �ڵ��������δӦ���޷���������ݵĹ�����˲�����������롣Ҫ�����������޷���������ݵĴ��룬�����ļ���ѡ���޷������Ԫ����Ȼ���������ɽű���

%% �����������
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

%% �����ʱ����
% clearvars filename startRow formatSpec fileID dataArray ans;