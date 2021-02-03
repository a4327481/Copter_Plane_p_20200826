% function  mat_to_matlab(varargin)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mat 格式转换为画图
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mat 格式转换为画图
global PathName1
if PathName1~=0
    cd(PathName1);
    [FileName1,PathName1,~] = uigetfile([PathName1,'\\*.*']); % 读取雷达数据
else
    [FileName1,PathName1,~] = uigetfile('*.*'); % 读取雷达数据
end
if FileName1==0
    return;
end
data1=importdata([PathName1,'\\',FileName1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ia=find(data1(2:end,1)==data1(1:end-1,1));
data1(ia,:)=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

t_imu2=data.IMU2(:,2)/1000;
wx2=data.IMU2(:,3)*57.3;
wy2=data.IMU2(:,4)*57.3;
wz2=data.IMU2(:,5)*57.3;

t_gps=data.GPS(:,14)/1000;
Lat=data.GPS(:,7);
Lng=data.GPS(:,8);
RelAlt=data.GPS(:,9);
Alt=data.GPS(:,10);
V_gps=data.GPS(:,11);
Gcrs=data.GPS(:,12);
Vz=data.GPS(:,13);

t_arsp=data.ARSP(:,2)/1000;
V_arsp=data.ARSP(:,3);


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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%F300-81#2
% ts1=(850:0.02:870)';
ts1=t_gps;
pitch_re_f=interp1(t_att,pitch_re,ts1);
V_f_arsp=interp1(t_arsp,V_arsp,ts1);
V_f_gps=interp1(t_gps,V_gps,ts1)/100;
pitch_f=interp1(t_att,pitch,ts1);
wz_f=interp1(t_imu,wz,ts1);

V_f_arsp_x=V_f_arsp./cosd(pitch_f);
V_f_arsp_x1=V_f_arsp./cosd(pitch_f+5);
V_f_arsp_x2=V_f_arsp./cosd(pitch_f-5);

%%%%%%%%%%%%%%%%%%%%
%耕地曲线摘取
Lat_m=data1(:,2);
Lng_m=data1(:,3);
Alt_m=data1(:,4);
Len_m=length(Lat_m);
ti=zeros(Len_m,1);

 is=4;
 [temp, ti(Len_m)]=min((Lat-Lat_m(Len_m)).^2+(Lng-Lng_m(Len_m)).^2); 
 [temp, ti(1)]=min((Lat-Lat_m(1)).^2+(Lng-Lng_m(1)).^2); 
 [temp, ti(2)]=min((Lat-Lat_m(2)).^2+(Lng-Lng_m(2)).^2); 
for i=3:Len_m-1
    [temp, ti(i)]=min((Lat(ti(2):ti(Len_m))-Lat_m(i)).^2+(Lng(ti(2):ti(Len_m))-Lng_m(i)).^2);    
end
isq=10/0.05;
ti(1:end-1)=ti(1:end-1)+ti(2);
i_n=[ti(is:4:end)+isq+16/0.05,ti(is+1:4:end)-isq];
i_p=[ti(is+2:4:end)+isq+16/0.05,ti(is+3:4:end)-isq];

i_n_len=length(i_n);
i_p_len=length(i_p);

V_wind_n=zeros(i_n_len,1);
V_wind_p=zeros(i_p_len,1);
V_f_gps_x=V_f_gps;
i_n1=[];
i_p1=[];
% plot(Lat,Lng,Lat_m,Lng_m,'*',Lat([i_n;i_p]),Lng([i_n;i_p]),'o')
figure
hold on
for i=1:i_n_len
% plot(Lat(i_n(i,1):i_n(i,2)),Lng(i_n(i,1):i_n(i,2)),'.')
plot(ts1(i_n(i,1):i_n(i,2)),V_f_arsp_x(i_n(i,1):i_n(i,2)))
plot(ts1(i_n(i,1):i_n(i,2)),V_f_gps(i_n(i,1):i_n(i,2)))

V_wind_n(i)=sum(V_f_arsp_x(i_n(i,1):i_n(i,2)))/(i_n(i,2)-i_n(i,1))-sum(V_f_gps(i_n(i,1):i_n(i,2))/(i_n(i,2)-i_n(i,1)));
V_f_gps_x(i_n(i,1):i_n(i,2))=V_f_gps(i_n(i,1):i_n(i,2))+V_wind_n(i);

plot(ts1(i_n(i,1):i_n(i,2))+4,V_f_gps_x(i_n(i,1):i_n(i,2)))

i_n1=[i_n1 ; (i_n(i,1):i_n(i,2))'];
end
figure
hold on
for i=1:i_p_len
% plot(Lat(i_n(i,1):i_n(i,2)),Lng(i_n(i,1):i_n(i,2)),'.')
plot(ts1(i_p(i,1):i_p(i,2)),V_f_arsp_x(i_p(i,1):i_p(i,2)))
plot(ts1(i_p(i,1):i_p(i,2)),V_f_gps(i_p(i,1):i_p(i,2)))

V_wind_p(i)=sum(V_f_arsp_x(i_p(i,1):i_p(i,2)))/(i_p(i,2)-i_p(i,1))-sum(V_f_gps(i_p(i,1):i_p(i,2))/(i_p(i,2)-i_p(i,1)));
V_f_gps_x(i_p(i,1):i_p(i,2))=V_f_gps(i_p(i,1):i_p(i,2))+V_wind_p(i);
plot(ts1(i_p(i,1):i_p(i,2))+4,V_f_gps_x(i_p(i,1):i_p(i,2)))

i_p1=[i_p1 ; (i_p(i,1):i_p(i,2))'];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if t_rc>ts1(1)
    j=1;
else
    j=find(t_rc<=ts1(1), 1, 'last' );    
end

len_ts0=length(ts1);
len_ts1=find(ts1<t_rc(end),1,'last');
delta_T_f=zeros(len_ts0,1);
delta_e_f=zeros(len_ts0,1);
flag=1;
for i=1:len_ts1-1
    while(flag)
        if(ts1(i)>=t_rc(j)&&ts1(i)<t_rc(j+1))
           delta_T_f(i)=delta_T(j);
           delta_e_f(i)=delta_e(j);
           flag=0;
        else
            j=j+1;
            flag=1;
        end
    end
    flag=1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_ck=[ts1 V_f_arsp V_f_arsp_x V_f_arsp_x1 V_f_arsp_x2 V_f_gps V_f_gps_x pitch_f wz_f delta_T_f delta_e_f];
fid=fopen([PathName,'\\','fz.dat'],'w');
fprintf(fid,'t Varsp V_f_arps_x V_f_arps_x1 V_f_arps_x2 Vgps V_f_gps_x pitch wz delat_T delat_e\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data_ck=[ts1 V_f_arsp V_f_arps_x V_f_arps_x1 V_f_arps_x2 V_f_gps pitch_f wz_f delta_T_f delta_e_f];
% fid=fopen([PathName,'\\','fz.dat'],'w');
% fprintf(fid,'t Varsp V_f_arps_x V_f_arps_x1 V_f_arps_x2 Vgps pitch wz delat_T delat_e\n');
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
%     fclose(fid);  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data_ck=[t_rc delta_a delta_e delta_T delta_r];
% fid=fopen([PathName,'\\','RCOUT.dat'],'w');
% fprintf(fid,'t 副翼 升降舵 油门 方向舵 \n');
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
%     fclose(fid);  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    