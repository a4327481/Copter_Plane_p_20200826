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
data=importdata([PathName,'\\',FileName]);
a=100;
b_imu=500;
at.imu=data.IMU(a+b_imu:end,2)/1000;
at.imu2=data.IMU(a+b_imu:end,2)/1000;
at.att=data.ATT(a:end,2)/1000;
at.arsp=data.ARSP(a:end,2)/1000;
at.baro=data.BARO(a:end,2)/1000;
at.ctun=data.CTUN(a:end,2)/1000;
at.gps=data.GPS(a:end,14)/1000;
at.sonar=data.SONAR(a:end,2)/1000;    
at.rcout=data.RCOUT(a:end,2)/1000; 
figure
hold on
grid on;
plot(at.imu(2:end),diff(at.imu),'-o')
plot(at.imu2(2:end),diff(at.imu2),'-o')
plot(at.att(2:end),diff(at.att),'-o')
plot(at.arsp(2:end),diff(at.arsp),'-o')
plot(at.baro(2:end),diff(at.baro),'-o')
plot(at.ctun(2:end),diff(at.ctun),'-o')
plot(at.gps(2:end),diff(at.gps),'-o')
plot(at.sonar(2:end),diff(at.sonar),'-o')
plot(at.rcout(2:end),diff(at.rcout),'-o')
legend('imu','imu2','att','arsp','baro','ctun','gps','sonar','rcout');
axis([650,1400,-1,1])
%  saveas(gcf,[PathName,'\\','t.dat'],'fig') ;
title('解算周期统计/diff(t)')
xlabel('t(s)')
ylabel('dt(s)')
 saveas(gcf,[PathName,'\\',FileName(1:end-6),'t.fig'],'fig') ;
 saveas(gcf,[PathName,'\\',FileName(1:end-6),'t.jpg'],'jpg') ;

% a=7800;
% % b=300;
% plot3(Lat(a:end),Lng(a:end),RelAlt(a:end),'-o')
% grid on
% xlabel('纬度')
% ylabel('经度')
% zlabel('高度')
