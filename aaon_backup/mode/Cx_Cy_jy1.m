% % 基本气动参数辨识
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %数据载入
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
HD=180/pi;
Sref=0.3;
t_rc=data.RCOUT(:,2)/1000;
delta_a=(data.RCOUT(:,3));
delta_e=(data.RCOUT(:,4));
delta_T=(data.RCOUT(:,5));
delta_r=(data.RCOUT(:,6));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_att=data.ATT(:,2)/1000;
roll=data.ATT(:,4)/100;
pitch_re=data.ATT(:,5)/100;
pitch=data.ATT(:,6)/100;
Yaw=data.ATT(:,8)/100;

t_imu=data.IMU(:,2)/1000;
wx=data.IMU(:,3)*57.3;
wy=data.IMU(:,4)*57.3;
wz=data.IMU(:,5)*57.3;
ax=data.IMU(:,6);
ay=data.IMU(:,7);
az=data.IMU(:,8);

t_imu2=data.IMU2(:,2)/1000;
wx2=data.IMU2(:,3)*57.3;
wy2=data.IMU2(:,4)*57.3;
wz2=data.IMU2(:,5)*57.3;
ax2=data.IMU2(:,6);
ay2=data.IMU2(:,7);
az2=data.IMU2(:,8);

t_gps=data.GPS(:,14)/1000;
Lat=data.GPS(:,7);
Lng=data.GPS(:,8);
RelAlt=data.GPS(:,9);
Alt=data.GPS(:,10);
V_gps=data.GPS(:,11)/100;
Gcrs=data.GPS(:,12)/100;
Vz=data.GPS(:,13);
V_gps_xyz=(V_gps.^2+Vz.^2).^0.5;
 
t_arsp=data.ARSP(:,2)/1000;
arsp=data.ARSP(:,3);
Diff_Py=data.ARSP(:,6);
len_P=length(Diff_Py);
Diff_P_offset=ones(len_P,1);
for i=2:len_P
    if(Diff_Py(i)==0)
        Diff_Py(i)=Diff_Py(i-1);
    end
end
Diff_P_offset(1)=0;
for i=4:10
    Diff_P_offset(1)= Diff_Py(i)+Diff_P_offset(1);
end
Diff_P_offset=ones(len_P,1)*Diff_P_offset(1)/7;
EAS=(abs(Diff_Py-Diff_P_offset)*2/1.225).^0.5;
T=data.ARSP(1,5)/100+ 273.15;
P=interp1(data.BARO(:,2)/1000,data.BARO(:,4),t_arsp);
% rho1=0.9962*ones(len_P,1);
rho1=1.225*288.15*P./T/101325;
TAS=(abs(Diff_Py-Diff_P_offset)*2./rho1).^0.5;
Diff_Py_abs=abs(Diff_Py-Diff_P_offset);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %样本1 E:\飞行试验\F300\F300_20181218搭载飞行试验\2018121901gf
% %时间选取 [897 958] 关油门段
% %阻力辨识
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% m=3.5;
% g=9.8;
% t_start=880;
% t_end=958;
% dt=0.020;
% ts_f=(t_start:dt:t_end)';
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q_f=interp1(t_arsp,Diff_Py_abs,ts_f);
% V_arps_f=interp1(t_arsp,TAS,ts_f);
% V_gps_xyz_f=interp1(t_gps,V_gps_xyz,ts_f);
% Vz_f=interp1(t_gps,Vz,ts_f);
% V_gps_f=interp1(t_gps,V_gps,ts_f);
% theta_f=-atand(Vz_f./V_gps_f);
% pitch_f=interp1(t_att,pitch,ts_f);
% alpha_f=pitch_f-theta_f;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %参数计算
% % dV_f=[0; diff(V_gps_xyz_f)]/dt;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %巴特沃斯滤波器
% fc = 1;   %截止频率
% fs = 50;  %采样频率
% n=6;        %阶次
% [b,a] = butter(n,fc/(fs/2));
% V_gps_xyz_f_1=filter(b,a,V_gps_xyz_f);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dV_f_1=[0; diff(V_gps_xyz_f_1)]/dt;
% X_f=m*dV_f_1+m*g*sind(theta_f);
% Cx_f=X_f./q_f ;
% 
% figure
% plot(ts_f ,Cx_f./alpha_f )
% title('Cx_f./alpha_f')
% figure
% plot(ts_f ,Cx_f)
% title('Cx_f')
% figure
% plot(ts_f, alpha_f)
% title('alpha_f')
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%样本2 E:\飞行试验\F300\F300_20181218搭载飞行试验\2018121901gf
%时间选取 [388 397] 全油门段
%油门辨识
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %时间截取和初始参数
% m=3.5;
% g=9.8;
% t_start=380;
% t_end=397;
% dt=0.020;
% ts_f=(t_start:dt:t_end)';
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q_f=interp1(t_arsp,Diff_Py_abs,ts_f);
% V_arps_f=interp1(t_arsp,TAS,ts_f);
% V_gps_xyz_f=interp1(t_gps,V_gps_xyz,ts_f);
% Vz_f=interp1(t_gps,Vz,ts_f);
% V_gps_f=interp1(t_gps,V_gps,ts_f);
% theta_f=-atand(Vz_f./V_gps_f);
% pitch_f=interp1(t_att,pitch,ts_f);
% alpha_f=pitch_f-theta_f;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %参数计算
% % dV_f=[0; diff(V_gps_xyz_f)]/dt;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %巴特沃斯滤波器
% fc = 1;   %截止频率
% fs = 50;  %采样频率
% n=6;        %阶次
% [b,a] = butter(n,fc/(fs/2));
% V_gps_xyz_f_1=filter(b,a,V_gps_xyz_f);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dV_f_1=[0; diff(V_gps_xyz_f_1)]/dt;
% X_f=q_f*0.05;
% T_f=m*dV_f_1+m*g*sind(theta_f)+X_f;
% figure
% plot(ts_f ,T_f)
% title('Tf')
% figure
% plot(ts_f, alpha_f)
% title('alpha')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 样本3 E:\飞行试验\F300\F300_20181218搭载飞行试验\2018121901gf
% 时间选取 [890 905] 关油门段
% 升力系数辨识
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%时间截取和初始参数
m=3.5;
g=9.8;
t_start=890;
t_end=905;
dt=0.020;
ts_f=(t_start:dt:t_end)';
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q_f=interp1(t_arsp,Diff_Py_abs,ts_f);
V_arps_f=interp1(t_arsp,TAS,ts_f);
V_gps_xyz_f=interp1(t_gps,V_gps_xyz,ts_f);
Vz_f=interp1(t_gps,Vz,ts_f);
V_gps_f=interp1(t_gps,V_gps,ts_f);
theta_f=-atand(Vz_f./V_gps_f);
pitch_f=interp1(t_att,pitch,ts_f);
alpha_f=pitch_f-theta_f;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%参数计算
% dV_f=[0; diff(V_gps_xyz_f)]/dt;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%巴特沃斯滤波器
fc = 1;   %截止频率
fs = 50;  %采样频率
n=6;        %阶次
[b,a] = butter(n,fc/(fs/2));
V_gps_xyz_f_1=filter(b,a,V_gps_xyz_f);
theta_f_1=filter(b,a,theta_f);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dV_f_1=[0; diff(V_gps_xyz_f_1)]/dt;
dtheta_f_1=[0;diff(theta_f_1)]/dt;
mvtheta=m*V_gps_xyz_f_1.*dtheta_f_1/57.3;
mgtheta=m*g*cosd(theta_f);
L_f=mvtheta+mgtheta;
Cy_S=L_f./q_f;
Cy_S_alpha=Cy_S./alpha_f;

figure
plot(ts_f ,L_f)
title('Lf')
figure
plot(ts_f, alpha_f)
title('alpha')
figure
plot(ts_f, Cy_S)
title('Cy_S')
figure
plot(ts_f, Cy_S_alpha)
title('Cy_S_alpha')

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 样本4 E:\飞行试验\F300\F300_20181218搭载飞行试验\2018122101
% % 时间选取 [625 646] 关油门段
% % 俯仰力矩系数辨识
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %时间截取和初始参数
% m=3.5;
% g=9.8;
% t_start=625;
% t_end=646;
% dt=0.020;
% ts_f=(t_start:dt:t_end)';
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q_f=interp1(t_arsp,Diff_Py_abs,ts_f);
% V_arps_f=interp1(t_arsp,TAS,ts_f);
% V_gps_xyz_f=interp1(t_gps,V_gps_xyz,ts_f);
% Vz_f=interp1(t_gps,Vz,ts_f);
% V_gps_f=interp1(t_gps,V_gps,ts_f);
% theta_f=-atand(Vz_f./V_gps_f);
% pitch_f=interp1(t_att,pitch,ts_f);
% roll_f=interp1(t_att,roll,ts_f);
% alpha_f=pitch_f-theta_f;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fs = 1/dt;           % Sampling frequency
% L = length(ts_f);      % Signal length
% n = 2^nextpow2(L);
% Y = fft(pitch_f,n);
% f = Fs*(0:(n/2))/n;
% P = abs(Y/n);
% plot(f,P(1:n/2+1)) 
 









