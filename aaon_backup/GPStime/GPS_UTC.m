%  % function  mat_to_matlab(varargin)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %mat 格式转换为画图
% global PathName
% if PathName~=0
%     cd(PathName);
%     [FileName,PathName,~] = uigetfile([PathName,'\\*.*']); % 读取雷达数据
% else
%     [FileName,PathName,~] = uigetfile('*.*'); % 读取雷达数据
% end
% if FileName==0
%     return;
% end
% data=importdata([PathName,'\\',FileName]);
time_week=data.GPS(1,4);
time_UTC=data.POS(:,2); 
ms_per_week = 7000 *86400 ;
unix_offset = 17000 *86400  + 52*10*7000 *86400  - 15000 ;
fix_time_ms = unix_offset +  time_week*ms_per_week ;
time_week_ms=time_UTC-unix_offset-time_week*ms_per_week;

% hold on
% plot(data.TECS(:,1),data.TECS(:,2),'O')
% plot(data.CTUN(:,1),data.CTUN(:,2),'*')
% plot(data.ATT(:,1),data.ATT(:,2),'*')
% plot(data.IMU(:,1),data.IMU(:,2),'*')

 