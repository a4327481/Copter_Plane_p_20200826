clc;clear;
warning off;
global PathName
if PathName~=0
    cd(PathName);
    [FileName,PathName,~] = uigetfile([PathName,'\\*.mat']); % 读取雷达数据
else
    [FileName,PathName,~] = uigetfile('*.mat'); % 读取雷达数据
end
if FileName==0
    return;
end
data=importdata([PathName,'\\',FileName]);

[FileName1,PathName1,~] = uigetfile([PathName,'\\*.dat']); % 读取雷达数据
data1=importdata([PathName1,'\\',FileName1]);


IMU_time=data.IMU(:,2)/1000;
IMU_Gyros=data.IMU(:,3:5);
IMU_accel=data.IMU(:,6:8);
 
IMU2_time=data.IMU2(:,2)/1000;
IMU2_Gyros=data.IMU2(:,3:5);
IMU2_accel=data.IMU2(:,6:8);
 
IMU3_time=data.IMU3(:,2)/1000;
IMU3_Gyros=data.IMU3(:,3:5);
IMU3_accel=data.IMU3(:,6:8);
 
% GPS_time=data.GPS(:,14)/1000;
% GPS_speed=data.GPS(:,11)/100;
% GPS_cog=data.GPS(:,12)/100;
% GPS_VZ=data.GPS(:,13);
% GPS_VX= GPS_speed.*cosd(GPS_cog);
% GPS_VY= GPS_speed.*sind(GPS_cog);
GPS_time=data1.data(:,1);
GPS_speed=data1.data(:,12);
GPS_VX= data1.data(:,5);
GPS_VY= data1.data(:,6);
GPS_VZ=data1.data(:,7);

ATT_time=data.ATT(:,2)/1000;
ATT_RollSensor=data.ATT(:,4)/100/180*pi;
ATT_PitchSensor=data.ATT(:,6)/100/180*pi;
ATT_YawSensor=data.ATT(:,8)/100/180*pi;

ARSP_time=data.ARSP(:,2)/1000;
ARSP_speed=data.ARSP(:,3);
% cog_arsp=interp1(GPS_time,GPS_cog,ARSP_time);
% % cog_arsp=interp1(ATT_time,ATT_YawSensor,ARSP_time);
% ARSP_speed_x=ARSP_speed.*cosd(cog_arsp);
% ARSP_speed_y=ARSP_speed.*sind(cog_arsp);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
Ts = 0.1;
t_start=GPS_time(1)+25;
% t_start=0;
t_end=GPS_time(end);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sim('err_rpy')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_ck=[an.time,[zeros(1,3); an.signals.values(1:end-1,:)],fn.signals.values,fb.signals.values,gn.signals.values,vxn.signals.values,vyn.signals.values,vzn.signals.values,ean.signals.values];
fid=fopen([PathName,'\\','data_out.dat'],'w');
fprintf(fid,'t anx any anz fnx fny fnz fbx fby fbz gnx gny gnz vxn vyn vzn eanx eany eanz\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);    
