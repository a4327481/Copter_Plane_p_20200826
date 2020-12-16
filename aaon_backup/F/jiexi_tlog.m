%%tloga 文件需要勾选ATT global_pos nav servo_out sys_sta（电压、电流、剩余电量） vfr
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
att_gps=importdata([PathName,'\\',FileName]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%时间轴选自 TimeMS IMU
yaw=att_gps.data(:,4);
len=length(yaw);
yaw1=yaw;
for i=1:len
    if yaw(i)<0
       yaw1(i)= yaw1(i)+360;
    end
end


data_ck=[att_gps.data(:,1)/1000,att_gps.data(:,2:3) yaw1 yaw att_gps.data(:,17:18) att_gps.data(:,36:43) att_gps.data(:,33)/1000 att_gps.data(:,34)/100 att_gps.data(:,35)];
fid=fopen([PathName,'\\','att_yc.dat'],'w');
fprintf(fid,'t roll pitch yaw1 yaw nav_roll nav_pitch airspeed groundspeed alt climb target_airspeed 油门 转速 里程 电压 电流 剩余mah\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i_t=20;
vx=att_gps.data(i_t:end,13);
vy=att_gps.data(i_t:end,14);
cog=atan2d(vy,vx);

len=length(cog);
cog1=cog;
for i=1:len
    if cog(i)<0
       cog1(i)= cog1(i)+360;
    end
end

data_ck=[att_gps.data(i_t:end,8)/1000,att_gps.data(i_t:end,9:10)*1e-7,att_gps.data(i_t:end,11:12)/1000,att_gps.data(i_t:end,13:15)/100,cog1];
fid=fopen([PathName,'\\','gps_yc.dat'],'w');
fprintf(fid,'t lat lon alt ralt vx vy vz cog\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_ck=[att_gps.data(:,23)/1000000,att_gps.data(:,24:28)];
fid=fopen([PathName,'\\','r_yc.dat'],'w');
fprintf(fid,'t 副翼 升降舵 油门 方向舵 开伞\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid); 

% Lat=att_gps.data(end,9)*1e-7
% Lng=att_gps.data(end,10)*1e-7
% dLng=-10/(Re/57.3.*cosd(Lat(1)))
% Xp=Re*Lat/57.3;
% Lat
% Lng1=Lng+dLng