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

% [FileName1,PathName1,~] = uigetfile([PathName,'\\*.*']); % 读取雷达数据
% data1=importdata([PathName1,'\\',FileName1]);
a=find(data.data(:,6)>5000);
data.data(a,:)=[];

i_star=find(data.data(:,2)>0, 1 );
ts=(data.data(i_star:end,2)-data.data(i_star,2)+data.data(i_star:end,3))/1000;

Re=6378137;
Lat=data.data(i_star:end,4);
Lng=data.data(i_star:end,5);
alt=data.data(i_star:end,6);
Xp=Re*Lng/57.3.*cosd(Lat(1));
Yp=Re*Lat/57.3;
Xp=Xp-Xp(1);
Yp=Yp-Yp(1);
V=((Xp(2:end)-Xp(1:end-1)).^2+(Yp(2:end)-Yp(1:end-1)).^2).^0.5/0.02;
Vz=(alt(2:end)-alt(1:end-1))/0.02;
V=[0 ;V];
fai=atan2(data.data(i_star:end,8),data.data(i_star:end,7))*57.3;
temp=find(fai<0);
fai(temp)=fai(temp)+360;
Vz=[0;Vz];
data_ck=[ts-ts(1),  data.data(i_star:end,4:end) Xp Yp V -Vz fai];
fid=fopen([PathName,'\\','GPS1.dat'],'w');
fprintf(fid,'t Lat纬度 Lng经度 高度 Vx Vy Vz NSats卫星数 RTK Xp Yp V Vz fai\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);      
  
    

