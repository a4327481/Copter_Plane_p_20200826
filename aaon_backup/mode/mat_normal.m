% % function  mat_to_matlab(varargin)
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % mat 格式转换为画图
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_rc=data.RCOUT(:,2)/1000;
delta_a=(data.RCOUT(:,3)-1500)/10;
delta_e=(data.RCOUT(:,4)-1500)/10;
delta_T=(data.RCOUT(:,5)-900)/1200;
delta_r=(data.RCOUT(:,6)-1500)/10;

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
ts1=(850:0.02:870)';

pitch_re_f=interp1(t_att,pitch_re,ts1);
V_f=interp1(t_arsp,V_arsp,ts1);
pitch_f=interp1(t_att,pitch,ts1);
wz_f=interp1(t_imu,wz,ts1);
j=find(t_rc<ts1(1), 1, 'last' );
len_ts1=length(ts1);
delta_T_f=zeros(len_ts1,1);
delta_e_f=zeros(len_ts1,1);
flag=1;
for i=1:len_ts1
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% data_ck=[ts1 V_f pitch_f wz_f delta_T_f delta_e_f];
% fid=fopen([PathName,'\\','fz.dat'],'w');
% fprintf(fid,'t V pitch wz delat_T delat_e\n');
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
%     fclose(fid);  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
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