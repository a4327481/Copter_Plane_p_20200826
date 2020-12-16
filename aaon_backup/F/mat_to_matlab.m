%  function  mat_to_matlab(varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%mat 格式转换为画图
global PathName
if PathName~=0
%     cd(PathName);
    [FileName,PathName,~] = uigetfile([PathName,'\\*.*']); % 读取雷达数据
else
    [FileName,PathName,~] = uigetfile('*.*'); % 读取雷达数据
end
if FileName==0
    return;
end
load([PathName,'\\',FileName]);
% importdata([PathName,'\\',FileName]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%时间轴选自 TimeMS IMU
data_ck=[IMU(:,2)/1000,IMU(:,3:5)*57.3,IMU(:,6:8)];
fid=fopen([PathName,'\\',FileName(1:end-6),'IMU.dat'],'w');
fprintf(fid,'t GyrX角速度wx GyrY角速度wy GyrZ角速度wz AccX加速度ax AccY加速度ay AccZ加速度az\n');
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid);
    save([PathName,'\\',FileName(1:end-6),'IMU.dat'],'data_ck','-ascii','-append' )

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%时间轴选自 TimeMS IMU2
data_ck=[IMU2(:,2)/1000,IMU2(:,3:5)*57.3,IMU2(:,6:8)];
fid=fopen([PathName,'\\',FileName(1:end-6),'IMU2.dat'],'w');
fprintf(fid,'t GyrX角速度wx GyrY角速度wy GyrZ角速度wz AccX加速度ax AccY加速度ay AccZ加速度az\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% %时间轴选自 TimeMS IMU3
data_ck=[IMU3(:,2)/1000,IMU3(:,3:5)*57.3,IMU3(:,6:8)];
fid=fopen([PathName,'\\',FileName(1:end-6),'IMU3.dat'],'w');
fprintf(fid,'t GyrX角速度wx GyrY角速度wy GyrZ角速度wz AccX加速度ax AccY加速度ay AccZ加速度az\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%时间轴选自 TimeMS ATT
%data_ck=[ATT(:,2)/1000,ATT(:,3:10)/100];
data_ck=[ATT(:,2)/1000,ATT(:,3:10)/100,ATT(:,11)];

fid=fopen([PathName,'\\',FileName(1:end-6),'ATT.dat'],'w');
%fprintf(fid,'t DesRoll标称滚转 Roll测量滚转 DesPitch标称俯仰 Pitch测量俯仰 DesYaw标称偏航 Yaw测量偏航 ErrRP俯仰滚转偏差 ErrYaw偏航误差 \n');
fprintf(fid,'t DesRoll标称滚转 Roll测量滚转 DesPitch标称俯仰 Pitch测量俯仰 DesYaw标称偏航 Yaw测量偏航 ErrRP俯仰滚转偏差 ErrYaw偏航误差 rpm动力电机转速\n');

[count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%时间轴选自 TimeMS ARSP 
data_ck=[ARSP(:,2)/1000,ARSP(:,3:4),ARSP(:,5)/100,ARSP(:,6:7)];
fid=fopen([PathName,'\\',FileName(1:end-6),'ARSP.dat'],'w');
fprintf(fid,'t Airspeed空速 DiffPress压差 Temp温度 RawPress压差原数 Offset偏置\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%时间轴选自 TimeMS BARO {'LineNo';'TimeMS';'Alt';'Press';'Temp';['CRt' char(13) '' char(10) '']}
temp0=ARSP(1,5)/100;
temp = temp0+ 273.15;
p1=(BARO(1,4)+BARO(2,4)+BARO(3,4)+BARO(4,4)+BARO(5,4))/5;
p2=BARO(:,4);
sc=p2/p1;
h_1=153.8462* temp .* (1.0 - sc.^0.190259);
h_2=29.271267/log10(exp(1))*(temp)*log10(1./sc);


data_ck=[BARO(:,2)/1000,BARO(:,3:4),BARO(:,5)/100,BARO(:,6),h_1,h_2];
fid=fopen([PathName,'\\',FileName(1:end-6),'BARO.dat'],'w');

fprintf(fid,'t Alt气压高度 Press压力 Temp温度 CRt H1 H2\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);   
 
  
%    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%时间轴选自 TimeMS CTUN    
data_ck=[CTUN(:,2)/1000,CTUN(:,3:6)/100,CTUN(:,7:10)];
fid=fopen([PathName,'\\',FileName(1:end-6),'CTUN.dat'],'w');
fprintf(fid,'t NavRoll导航滚转 Roll滚转角 NavPitch导航俯仰 Pitch俯仰 ThrOut总能量T RdrOut总能量T AccY加速度 xtrack_error\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid); 
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%时间轴选自 TimeMS   GPS {'LineNo';'Status';'TimeMS';'Week';'NSats';'HDop';'Lat';'Lng';'RelAlt';'Alt';'Spd';'GCrs';'VZ';['T' char(13) '' char(10) '']}
ik=find(GPS(:,11)/100>100);
GPS(ik,:)=[];
% ik=22410;
% GPS(ik:end,:)=[];
Re=6378137;
Fe=1.0/298.257223563;
e2=Fe*(2-Fe);
Lat=GPS(:,7)*1e-7;%纬度
Lng=GPS(:,8)*1e-7;%经度
RelAlt=GPS(:,9)/100;%气压高
Alt=GPS(:,10)/100;%高度
N=Re./(1-e2*sind(Lat).*sind(Lat)).^0.5;

X=(N+Alt).*cosd(Lat).*cosd(Lng);
Y=(N+Alt).*cosd(Lat).*sind(Lng);
Z=(N*(1-e2)+Alt).*sind(Lat);

len=length(X);
Xg=zeros(len,1);
Yg=zeros(len,1);
Zg=zeros(len,1);
for i=1:1
% c=[-sind(Lng(i)) cosd(Lng(i)) 0
%    -sind(Lat(i))*cosd(Lng(i)) -sind(Lat(i))*sind(Lng(i)) cosd(Lat(i))
%     cosd(Lat(i))*cosd(Lng(i))  cosd(Lat(i))*sind(Lng(i)) sind(Lat(i))];
c=[-sind(Lat(i))*cosd(Lng(i)) -sind(Lat(i))*sind(Lng(i)) cosd(Lat(i))
   -sind(Lng(i))                cosd(Lng(i))                    0
   -cosd(Lat(i))*cosd(Lng(i)) -cosd(Lat(i))*sind(Lng(i)) -sind(Lat(i))];


% cx=c*([X(i) Y(i) Z(i)]');
cx=c*([X  Y   Z ]');

Xg=cx(1,:)';
Yg=cx(2,:)';
Zg=cx(3,:)';
end
Xg=Xg-Xg(1);
Yg=Yg-Yg(1);
Zg=-(Zg-Zg(1));

Yp=Re*Lng/57.3.*cosd(Lat(1));
Xp=Re*Lat/57.3;
% Lat0=38.0975341796875; 
% Lng0=114.645797729492;
% Yp0=Re*Lng0/57.3.*cosd(Lat(1));
% Xp0=Re*Lat0/57.3;

Xp=Xp-Xp(1);
Yp=Yp-Yp(1);
% Xp=Xp-Xp0;
% Yp=Yp-Yp0;
V=((GPS(:,11)/100).^2+GPS(:,13).^2).^0.5;
Vx=(GPS(:,11)/100).*cosd(GPS(:,12)/100);
Vy=(GPS(:,11)/100).*sind(GPS(:,12)/100);
theta=-atand(GPS(:,13)./V);
% len=800;
% plot(Yp(1:len),Xp(1:len),'LineWidth',2)
% figure
% plot3(Yp(1:len),Xp(1:len),RelAlt(1:len),'LineWidth',2)

% len=800;
% a=100;
% hold on
% for i=0:a:len
%     lena=i+a;
%     lenb=i;
%     
% plot3(Yp(end-lena:end-lenb),Xp(end-lena:end-lenb),RelAlt(end-lena:end-lenb),'LineWidth',2)
% end

data_ck=[GPS(:,14)/1000,GPS(:,2),GPS(:,5:6),GPS(:,7:8)*1e-7,GPS(:,9:11)/100,GPS(:,12)/100,GPS(:,13),X,Y,Z,Xp,Yp,V,Vx,Vy,theta Xg Yg Zg];%
fid=fopen([PathName,'\\',FileName(1:end-6),'GPS.dat'],'w');
fprintf(fid,'t state NSats卫星数 HDop Lat纬度 Lng经度 RelAlt气压高度 Alt高度 Spd卫星速度 Gcrs航向 Vz垂直速度 X Y Z Xp Yp V Vx Vy theta Xg Yg Zg\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);      
 
%     
% data_ck=[GPS(:,14)/1000-GPS(1,14)/1000,GPS(:,5:6),GPS(:,7:8)*1e-7,GPS(:,9:11)/100,GPS(:,12)/100,GPS(:,13),X,Y,Z,Xp,Yp,((GPS(:,11)/100).^2+GPS(:,13).^2).^0.5];
% fid=fopen([PathName,'\\',FileName(1:end-6),'GPS0.dat'],'w');
% fprintf(fid,'t NSats卫星数 HDop Lat纬度 Lng经度 RelAlt气压高度 Alt高度 Spd卫星速度 Gcrs航向 Vz垂直速度 X Y Z Xp Yp V\n');
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
%     fclose(fid);      
%      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%时间轴选自 TimeMS   CURR {'LineNo';'TimeMS';'Volt';'Curr';'RemPct';'Temp';'RemCap';'cell1';'cell2';'cell3';['cell4' char(13) '' char(10) '']}
data_ck=[CURR(:,2)/1000,CURR(:,4:5),CURR(:,9:10),CURR(:,15:19)];
fid=fopen([PathName,'\\',FileName(1:end-6),'CURR.dat'],'w');
fprintf(fid,'t Volt Curr RemCap temp  RemPct cell1 cell2 cell3 cell4\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);      
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%时间轴选自 TimeMS   SONAR {'LineNo';'TimeMS';'DistCM';'Volt';'BaroAlt';'GSpd';'Thr';'Cnt';['Corr' char(13) '' char(10) '']}
data_ck=[SONAR(:,2)/1000,SONAR(:,3)/100];
fid=fopen([PathName,'\\',FileName(1:end-6),'SONAR.dat'],'w');
fprintf(fid,'t DistCM机载高度表\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);     
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%输出 
data_ck=[RCOUT(:,2)/1000,RCOUT(:,3:7)/100];
fid=fopen([PathName,'\\',FileName(1:end-6),'RCOUT.dat'],'w');
fprintf(fid,'t 副翼 升降舵 油门 方向舵 开伞\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);     
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%状态 
data_ck=[STS(:,2)/1000,STS(:,5)];
fid=fopen([PathName,'\\',FileName(1:end-6),'STS.dat'],'w');
fprintf(fid,'t 异常标识\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%TECS
data_ck=[TECS(:,2)/1e3,TECS(:,3:12) TECS(:,13)*57.3 TECS(:,14:16)];
fid=fopen([PathName,'\\',FileName(1:end-6),'TECS.dat'],'w');
fprintf(fid,'t height climb_rate hgt_dem_adj hgt_rate_dem TAS_dem_adj TAS_state vel_dot integTHR_state integSEB_state throttle_dem pitch_dem TAS_rate_dem ste_error seb_error\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%NTUM {'LineNo';'TimeMS';'Yaw';'WpDist';'TargBrg';'NavBrg';'AltErr';'Arspd';'Alt';'GSpdCM';'windx';['windy' char(13) '' char(10) '']}
yaw=NTUN(:,3)/100;
iy=find(yaw>180);
yaw(iy)=yaw(iy)-360;
data_ck=[NTUN(:,2)/1e3,NTUN(:,3:6)/100,NTUN(:,7:12) yaw];
fid=fopen([PathName,'\\',FileName(1:end-6),'NTUM.dat'],'w');
fprintf(fid,'t  Yaw WpDist TargBrg NavBrg AltErr Arspd Alt GSpdCM windx windy yaw_x\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%POS
time_week=GPS(1,4);
time_UTC=POS(:,2); 
ms_per_week = 7000 *86400 ;
unix_offset = 17000 *86400  + 52*10*7000 *86400  - 15000 ;
fix_time_ms = unix_offset +  time_week*ms_per_week ;
time_week_ms=time_UTC-unix_offset-time_week*ms_per_week;
ir=find(GPS(2:end,3)==GPS(1:end-1,3));
GPS(ir,:)=[];
t_pos=interp1(GPS(:,3),GPS(:,14),time_week_ms);
data_ck=[t_pos/1000,POS(:,3),POS(:,4)*1e-7,POS(:,5)*1e-7,POS(:,6:7)/1000,POS(:,8:10)/100];
fid=fopen([PathName,'\\',FileName(1:end-6),'POS.dat'],'w');
fprintf(fid,'t ID Lat纬度 Lng经度 Alt高度 RelAlt Roll Pitch Yaw\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid); 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MODE

 data_ck=[MODE(:,2)/1e3,MODE(:,3)];
fid=fopen([PathName,'\\',FileName(1:end-6),'MODE.dat'],'w');
fprintf(fid,'t  MODE\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);   
%  end
%     %POS
% data_ck=[(POS(:,2))/1e3,POS(:,2).*0+0.04,POS(:,3)];
% fid=fopen([PathName,'\\',FileName(1:end-6),'POS.dat'],'w');
% fprintf(fid,'t POS ID\n');
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
%     fclose(fid); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   