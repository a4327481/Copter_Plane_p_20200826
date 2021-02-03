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
t_rc=data.RCOUT(:,2)/1000;
delta_a=(data.RCOUT(:,3)-1500)/10;
delta_e=(data.RCOUT(:,4)-1500)/10;
delta_T=(data.RCOUT(:,5)-900)/1200;
delta_r=(data.RCOUT(:,6)-1500)/10;
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
ax=data.IMU2(:,6);
ay=data.IMU2(:,7);
az=data.IMU2(:,8);

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
V_arsp=data.ARSP(:,3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%时间轴选自 TimeMS ARSP 
Diff_Py=data.ARSP(:,6);
t_ARSP=data.ARSP(:,2)/1000;
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
EAS=((Diff_Py-Diff_P_offset)*2/1.225).^0.5;
T=data.ARSP(1,5)/100+ 273.15;
P=interp1(data.BARO(:,2)/1000,data.BARO(:,4),t_ARSP);

% rho1=0.9962*ones(len_P,1);
rho1=1.225*288.15*P./T/101325;
TAS=((Diff_Py-Diff_P_offset)*2./rho1).^0.5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Re=6378137;
Fe=1.0/298.257223563;
e2=Fe*(2-Fe);
Lat=data.GPS(:,7)*1e-7;%纬度
Lng=data.GPS(:,8)*1e-7;%经度
RelAlt=data.GPS(:,9)/100;%气压高
Alt=data.GPS(:,10)/100;%高度
N=Re./(1-e2*sind(Lat).*sind(Lat)).^0.5;

X=(N+Alt).*cosd(Lat).*cosd(Lng);
Y=(N+Alt).*cosd(Lat).*sind(Lng);
Z=(N*(1-e2)+Alt).*sind(Lat);

Xp=Re*Lng/57.3.*cosd(Lat(1));
Yp=Re*Lat/57.3;
Xp=Xp-Xp(1);
Yp=Yp-Yp(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_imu2=data.IMU2(:,2)/1000;
ax_imu2=data.IMU2(:,6);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%F300-81#2
% ts1=(850:0.02:870)';
ts1=t_gps;
pitch_re_f=interp1(t_att,pitch_re,ts1);
V_f_arsp=interp1(t_arsp,V_arsp,ts1)-1;
V_f_gps=interp1(t_gps,V_gps,ts1);
V_f_gps_xyz=interp1(t_gps,V_gps_xyz,ts1);

wz_f=interp1(t_imu,wz,ts1);
wy_f=interp1(t_imu,wy,ts1);
wx_f=interp1(t_imu,wx,ts1);
ax_f=interp1(t_imu,ax,ts1);
ay_f=interp1(t_imu,ay,ts1);
az_f=interp1(t_imu,az,ts1);

roll_f=interp1(t_att,roll,ts1);
pitch_f=interp1(t_att,pitch,ts1);
yaw_f=interp1(t_att,Yaw,ts1);




V_f_arsp_x=V_f_arsp./cosd(pitch_f);
V_f_arsp_x1=V_f_arsp./cosd(pitch_f+5);
V_f_arsp_x2=V_f_arsp./cosd(pitch_f-5);
V_f_gps_x=V_f_gps;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% t.in=[460 470;545 558;630 641];
% t.ip=[498 522;583 610;668 690];
% t.in_len=length(t.in);
% t.ip_len=length(t.ip);
% t.in_i=zeros(t.in_len,2);
% t.ip_i=zeros(t.in_len,2);
% 
% t.V_wind_n=zeros(t.in_len,1);
% t.V_wind_p=zeros(t.ip_len,1);
% i_n1=[];
% i_p1=[];
% figure
% hold on
% for i=1:t.in_len
%     [~, t.in_i(i,1)]=min(abs(t.in(i,1)-ts1));
%     [~, t.in_i(i,2)]=min(abs(t.in(i,2)-ts1));
%     t.V_wind_n(i)=sum(V_f_arsp(t.in_i(i,1):t.in_i(i,2)))/(t.in_i(i,2)-t.in_i(i,1))-sum(V_f_gps(t.in_i(i,1):t.in_i(i,2)))/(t.in_i(i,2)-t.in_i(i,1));
%     V_f_gps_x(t.in_i(i,1):t.in_i(i,2))= V_f_gps(t.in_i(i,1):t.in_i(i,2))+t.V_wind_n(i);
% 
%     plot(ts1(t.in_i(i,1):t.in_i(i,2)),V_f_arsp(t.in_i(i,1):t.in_i(i,2)))
%     plot(ts1(t.in_i(i,1):t.in_i(i,2)),V_f_gps(t.in_i(i,1):t.in_i(i,2)))
%     plot(ts1(t.in_i(i,1):t.in_i(i,2)),V_f_gps_x(t.in_i(i,1):t.in_i(i,2)))
% 
% end
% 
% figure
% hold on
% for i=1:t.ip_len
%     [~, t.ip_i(i,1)]=min(abs(t.ip(i,1)-ts1));
%     [~, t.ip_i(i,2)]=min(abs(t.ip(i,2)-ts1));
%     t.V_wind_p(i)=sum(V_f_arsp(t.ip_i(i,1):t.ip_i(i,2)))/(t.ip_i(i,2)-t.ip_i(i,1))-sum(V_f_gps(t.ip_i(i,1):t.ip_i(i,2)))/(t.ip_i(i,2)-t.ip_i(i,1));
%     V_f_gps_x(t.ip_i(i,1):t.ip_i(i,2))= V_f_gps(t.ip_i(i,1):t.ip_i(i,2))+t.V_wind_p(i);
%     
%     plot(ts1(t.ip_i(i,1):t.ip_i(i,2)),V_f_arsp(t.ip_i(i,1):t.ip_i(i,2)))
%     plot(ts1(t.ip_i(i,1):t.ip_i(i,2)),V_f_gps(t.ip_i(i,1):t.ip_i(i,2)))
%     plot(ts1(t.ip_i(i,1):t.ip_i(i,2)),V_f_gps_x(t.ip_i(i,1):t.ip_i(i,2)))
% end
% 
% % hold on
% % plot(ts1,V_f_gps)
% % plot(ts1,V_f_gps_xyz)
% 
% % plot(t_imu2,ax_imu2+16)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% t.iy=[737 800;875 900];
% t.iy_len=length(t.iy);
% t.iy_i=zeros(t.iy_len,2);
% t.V_wind_n=zeros(t.iy_len,1);
% i_y1=[];
% 
% 
% % figure
% % hold on
% % for i=1:t.iy_len
%     i=1;
%     [~, t.iy_i(i,1)]=min(abs(t.iy(i,1)-ts1));
%     [~, t.iy_i(i,2)]=min(abs(t.iy(i,2)-ts1));
%     itemp=find(abs(Gcrs(t.iy_i(i,1)+10:t.iy_i(i,2))-Gcrs(t.iy_i(i,1)))<0.3)
%     temp_V_x=0;
%     temp_V_y=0;
%     temp_Va_x=0;
%     temp_Va_y=0;
%     temp=0;
%     temp1=0;
%     i_temp0=zeros(10+itemp(1),1);
%     for j=t.iy_i(i,1):t.iy_i(i,1)+10+itemp(1)
%         dGcrs=Gcrs(j+1)-Gcrs(j);
%         if abs(dGcrs)>=20
%             dGcrs=dGcrs+360;
%         end
%         i_temp0(j-t.iy_i(i,1)+1)=dGcrs;
%         temp=temp+cosd(Gcrs(j))*dGcrs;
%         temp1=temp1+dGcrs;
%         temp_V_x=V_f_gps(j)*cosd(Gcrs(j))*dGcrs+temp_V_x;  
%         temp_V_y=V_f_gps(j)*sind(Gcrs(j))*dGcrs+temp_V_y;
%         temp_Va_x=V_f_arsp(j)*cosd(Gcrs(j))*dGcrs+temp_Va_x;  
%         temp_Va_y=V_f_arsp(j)*sind(Gcrs(j))*dGcrs+temp_Va_y;
% %         temp_Va_x=cosd(Gcrs(j))*dGcrs+temp_Va_x;  
% %         temp_Va_y=18*sind(Gcrs(j))*dGcrs+temp_Va_y;   
% % hold on
% % plot(V_f_arsp(j),'o')
%     end
%     
%     temp_V_x=temp_V_x/360
%     temp_V_y=temp_V_x/360
%     temp_Va_x=temp_Va_x
%     temp_Va_y=temp_Va_y
% %         temp_V_x=temp_V_x/(ts1(t.iy_i(i,1)+10+itemp(1))-ts1(t.iy_i(i,1)))  
% %         temp_V_y=temp_V_y/(ts1(t.iy_i(i,1)+10+itemp(1))-ts1(t.iy_i(i,1)))
% %         temp_Va_x=temp_Va_x/(ts1(t.iy_i(i,1)+10+itemp(1))-ts1(t.iy_i(i,1))) 
% %         temp_Va_y=temp_Va_y/(ts1(t.iy_i(i,1)+10+itemp(1))-ts1(t.iy_i(i,1)))
%         
% % temp_Va_x*cosd(338)+temp_Va_y*sind(338)
%         
% Vx_f_gps=V_f_gps.*cosd(Gcrs);
% Vy_f_gps=V_f_gps.*sind(Gcrs);
% Vx_f_arsp=V_f_arsp.*cosd(Gcrs);
% Vy_f_arsp=V_f_arsp.*sind(Gcrs);
% for i=100:len_dcm-100
%     V_f_gps_x_v(i,1)=Vx_f_arsp(i-99)+sum(dV(i-99:i-1,1));
% end
% Vwx=Vx_f_gps-Vx_f_arsp;
% Vwy=Vy_f_gps-Vy_f_arsp;
% i=1;
% hold on
% plot(ts1,Vx_f_arsp,ts1,V_f_gps_x_v(:,1),'bla')
% % plot(Gcrs(t.iy_i(i,1):t.iy_i(i,1)+10+itemp(1)))
% 
% % plot(Gcrs(t.iy_i(i,1):t.iy_i(i,1)+10+itemp(1))-Gcrs(t.iy_i(i,1)-1:t.iy_i(i,1)+10+itemp(1)-1))
% 
% a=t.iy_i(i,1):t.iy_i(i,1)+10+itemp(1);
% plot(ts1(a),V_f_gps(a),ts1(a),V_f_arsp(a))
% plot(ts1(a),Vx_f_gps(a),ts1(a),Vy_f_gps(a),ts1(a),V_f_gps(a))
% hold on
% plot(ts1(a),Vx_f_arsp(a),ts1(a),Vy_f_arsp(a),ts1(a),V_f_arsp(a))
% 
% figure
% hold on
% grid on
% plot(Vx_f_arsp(a),Vy_f_arsp(a))
% plot(Vx_f_gps(a),Vy_f_gps(a))
% plot(Vwx(a),Vwy(a))
% legend('arsp','gps','wind')
% plot(0,0,'o')
% plot(Vx_f_arsp(a(1)),Vy_f_arsp(a(1)),'*')
% plot(Vx_f_gps(a(1)),Vy_f_gps(a(1)),'*')
% plot(Vwx(a(1)),Vwy(a(1)),'*')
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  
% a1=t.iy_i(i,1)+10+itemp(1):(t.iy_i(i,1)+10+itemp(2)-600);
% % plot(ts1(a1),V_f_gps(a1),ts1(a1),V_f_arsp(a1))
% % plot(ts1(a1),Vx_f_gps(a1),ts1(a1),Vy_f_gps(a1),ts1(a1),V_f_gps(a1))
% % hold on
% % plot(ts1(a1),Vx_f_arsp(a1),ts1(a1),Vy_f_arsp(a1),ts1(a1),V_f_arsp(a1))
% 
% figure
% hold on
% grid on
% plot(Vx_f_arsp(a1),Vy_f_arsp(a1))
% plot(Vx_f_gps(a1),Vy_f_gps(a1))
% plot(Vwx(a1),Vwy(a1))
% legend('arsp','gps','wind')
% plot(0,0,'o')
% plot(Vx_f_arsp(a1(1)),Vy_f_arsp(a1(1)),'*')
% plot(Vx_f_gps(a1(1)),Vy_f_gps(a1(1)),'*')
% plot(Vwx(a1(1)),Vwy(a1(1)),'*')
% 
% % end
% %     t.iy_i(i,3)=find((Gcrs(t.iy_i(i,1):t.iy_i(i,2))-Gcrs(t.iy_i(i,1)))<0,1,'first')
% %     t.iy_i(i,4)=find((Gcrs(t.iy_i(i,3):t.iy_i(i,2))-Gcrs(t.iy_i(i,3)))<0,1,'first')
% 
% %  
% % figure
% % hold on
% % for i=1:t.ip_len
% %     [~, t.ip_i(i,1)]=min(abs(t.ip(i,1)-ts1));
% %     [~, t.ip_i(i,2)]=min(abs(t.ip(i,2)-ts1));
% %     t.V_wind_p(i)=sum(V_f_arsp(t.ip_i(i,1):t.ip_i(i,2)))/(t.ip_i(i,2)-t.ip_i(i,1))-sum(V_f_gps(t.ip_i(i,1):t.ip_i(i,2)))/(t.ip_i(i,2)-t.ip_i(i,1));
% %     V_f_gps_x(t.ip_i(i,1):t.ip_i(i,2))= V_f_gps(t.ip_i(i,1):t.ip_i(i,2))+t.V_wind_p(i);
% %     
% %     plot(ts1(t.ip_i(i,1):t.ip_i(i,2)),V_f_arsp(t.ip_i(i,1):t.ip_i(i,2)))
% %     plot(ts1(t.ip_i(i,1):t.ip_i(i,2)),V_f_gps(t.ip_i(i,1):t.ip_i(i,2)))
% %     plot(ts1(t.ip_i(i,1):t.ip_i(i,2)),V_f_gps_x(t.ip_i(i,1):t.ip_i(i,2)))
% % end
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% len_dcm=length(roll_f);
% dt=[0 ;ts1(2:end)-ts1(1:end-1)];
% DCM{len_dcm,1}=[];
% dV=zeros(len_dcm,3);
% V_f_gps_x_v=zeros(len_dcm,3);
% V_f_gps_x_ax=zeros(len_dcm,3);
% dVsf=zeros(len_dcm,3);
% gn=[0 0 9.8]';
% dVt=zeros(len_dcm,3);
% for i=1:len_dcm   
% rz=[cosd(yaw_f(i))    -sind(yaw_f(i))  0;  sind(yaw_f(i)) cosd(yaw_f(i))  0;0 0 1];
% rz=1;
% ry=[cosd(pitch_f(i))   0  sind(pitch_f(i)); 0              1               0;-sind(pitch(i)) 0 cosd(pitch(i))];
% rx=[1                     0               0;0 cosd(roll_f(i)) -sind(roll_f(i));0 sind(roll_f(i)) cosd(roll_f(i))];
% DCM{i}=rz*ry*rx;
% dVt(i,:)=[ax_f(i) ay_f(i) az_f(i)]*dt(i);
% % dVt(i,:)=[ax_f(i) ay_f(i) az_f(i)];
% dQt=[wx_f(i) wy_f(i) wz_f(i)]*0.02/57.3;
% dVsf(i,:)=(DCM{i}*(dVt(i,:)+0.5*cross(dQt,dVt(i,:)))')';
% %  dVsf(i,:)=((dVt(i,:)+0.5*cross(dQt,dVt(i,:)))')';
% dV(i,:)=(dVsf(i,:)'+gn*dt(i))';
% end
% % plot(ts1,dVsf(:,1),ts1,dVt(:,1))
% % ,ts1,ax_f.*0.02,'r')
% 
% Vx_f_gps=V_f_gps.*cosd(Gcrs);
% Vy_f_gps=V_f_gps.*sind(Gcrs);
% Vx_f_arsp=V_f_arsp.*cosd(Gcrs);
% Vy_f_arsp=V_f_arsp.*sind(Gcrs);
% nj=100;
% for i=nj:len_dcm-nj
%     V_f_gps_x_v(i+1,1)=V_f_arsp(i-nj+1)+sum(dV(i-nj+1:i,1));
% %     V_f_gps_x_ax(i+1,1)=V_f_arsp(i-nj+1)+sum(ax_f(i-nj+1:i,1))*0.02;
%     
% end
% Vwx=Vx_f_gps-Vx_f_arsp;
% Vwy=Vy_f_gps-Vy_f_arsp;
% i=1;
% hold on
% plot(ts1,V_f_arsp,ts1,V_f_gps_x_v(:,1))
% % plot(ts1,dV(:,1))
% % plot(ts1,V_f_arsp,ts1,V_f_gps_x_ax,'bla')
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% hold on
% plot(ts1,V_f_gps)
% plot(ts1,V_f_gps_xyz)
% Vx_f_gps_w=Vx_f_gps+4.5;
% Vy_f_gps_w=Vy_f_gps-4.1;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data_ck=[ts1 V_f_arsp V_f_arsp_x V_f_arsp_x1 V_f_arsp_x2 V_f_gps V_f_gps_x pitch_f wz_f Vx_f_gps Vy_f_gps Vx_f_arsp Vy_f_arsp Vwx Vwy (Vwx.^2+Vwy.^2).^0.5 Vx_f_gps_w Vy_f_gps_w ax_f-9.8*sind(pitch_f)];
% fid=fopen([PathName,'\\','fz1.dat'],'w');
% fprintf(fid,'t Varsp V_f_arps_x V_f_arps_x1 V_f_arps_x2 Vgps V_f_gps_x pitch wz Vx_f_gps Vy_f_gps Vx_f_arsp Vy_f_arsp Vwx Vwy Vw Vx_f_gps_w Vy_f_gps_w ax_f_x\n');
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
%     fclose(fid);  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%