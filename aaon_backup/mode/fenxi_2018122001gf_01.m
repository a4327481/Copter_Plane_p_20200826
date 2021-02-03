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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_rc=data.RCOUT(:,2)/1000;
delta_a=(data.RCOUT(:,3) );%副翼
delta_e=(data.RCOUT(:,4) );%升级
delta_T=(data.RCOUT(:,5) );%油门
delta_r=(data.RCOUT(:,6) );%方向
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_att=data.ATT(:,2)/1000;
roll=data.ATT(:,4)/100;
pitch_re=data.ATT(:,5)/100;
pitch=data.ATT(:,6)/100;
Yaw=data.ATT(:,8)/100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_imu=data.IMU(:,2)/1000;
wx=data.IMU(:,3)*57.3;
wy=data.IMU(:,4)*57.3;
wz=data.IMU(:,5)*57.3;
ax=data.IMU(:,6);
ay=data.IMU(:,7);
az=data.IMU(:,8);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_imu2=data.IMU2(:,2)/1000;
wx2=data.IMU2(:,3)*57.3;
wy2=data.IMU2(:,4)*57.3;
wz2=data.IMU2(:,5)*57.3;
ax2=data.IMU2(:,6);
ay2=data.IMU2(:,7);
az2=data.IMU2(:,8);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ik=find(data.GPS(:,11)/100>100);
data.GPS(ik,:)=[];

t_gps=data.GPS(:,14)/1000;
V_h_gps=data.GPS(:,11)/100;
Gcrs=data.GPS(:,12)/100;
Vz=data.GPS(:,13);
Re=6378137;
Fe=1.0/298.257223563;
e2=Fe*(2-Fe);
Lat=data.GPS(:,7)*1e-7;%纬度
Lng=data.GPS(:,8)*1e-7;%经度
RelAlt=data.GPS(:,9)/100;%气压高
Alt=data.GPS(:,10)/100;%高度
N=Re./(1-e2*sind(Lat).*sind(Lat)).^0.5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X=(N+Alt).*cosd(Lat).*cosd(Lng);
Y=(N+Alt).*cosd(Lat).*sind(Lng);
Z=(N*(1-e2)+Alt).*sind(Lat);

len=length(X);
Xg=zeros(len,1);
Yg=zeros(len,1);
Zg=zeros(len,1);
for i=1:1
 c=[-sind(Lat(i))*cosd(Lng(i)) -sind(Lat(i))*sind(Lng(i)) cosd(Lat(i))
   -sind(Lng(i))                cosd(Lng(i))                    0
   -cosd(Lat(i))*cosd(Lng(i)) -cosd(Lat(i))*sind(Lng(i)) -sind(Lat(i))];
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
Xp=Xp-Xp(1);
Yp=Yp-Yp(1);
V_gps=((data.GPS(:,11)/100).^2+data.GPS(:,13).^2).^0.5;
Vx=(data.GPS(:,11)/100).*cosd(data.GPS(:,12)/100);
Vy=(data.GPS(:,11)/100).*sind(data.GPS(:,12)/100);
theta=-atand(data.GPS(:,13)./V_gps); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_arsp=data.ARSP(:,2)/1000;
V_arsp=data.ARSP(:,3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ts1=(t_gps(1):0.02:t_gps(end))';
ts1=t_gps;
V_arsp_f=interp1(t_arsp,V_arsp,ts1);
V_gps_f=interp1(t_gps,V_gps,ts1);
V_h_gps_f=interp1(t_gps,V_h_gps,ts1);
Gcrs_f=interp1(t_gps,Gcrs,ts1,'linear');
wz_f=interp1(t_imu,wz,ts1);
wy_f=interp1(t_imu,wy,ts1);
wx_f=interp1(t_imu,wx,ts1);
ax_f=interp1(t_imu,ax,ts1);
ay_f=interp1(t_imu,ay,ts1);
az_f=interp1(t_imu,az,ts1);

roll_f=interp1(t_att,roll,ts1);
pitch_f=interp1(t_att,pitch,ts1);
yaw_f=interp1(t_att,Yaw,ts1);
theta_f=interp1(t_gps,theta,ts1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
temp0=data.ARSP(1,5)/100;
T = temp0+ 273.15;
P=(data.BARO(1,4)+data.BARO(2,4)+data.BARO(3,4)+data.BARO(4,4)+data.BARO(5,4))/5;
rho = 1.225*288.15*P/T/101325;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Cx 估算 
% 计算方法 m*diff(V_gps)=T-D-m*g*sin(theta)
% 令T=0;则 m*diff(V_gps)=-D-m*g*sin(theta)
% 令D=0.5*rho*V_arsp*V_arsp*Cx;则  m*diff(V_gps)=-0.5*rho*V_arsp*V_arsp*Cx-m*g*sin(theta)
% m*diff(V_gps)+m*g*sin(theta)=-0.5*rho*V_arsp*V_arsp*Cx(alpha,V)
plot(ts1(2:1:end),diff(V_gps_f(1:1:end))./(ts1(2:1:end)-ts1(1:end-1)),ts1,ax_f)




