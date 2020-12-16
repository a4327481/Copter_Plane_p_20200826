% function  mat_to_matlab(varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%mat 格式转换为画图
global PathName
if PathName~=0
    cd(PathName);
    [FileName,PathName,~] = uigetfile([PathName,'\\*.txt']); % 读取雷达数据
else
    [FileName,PathName,~] = uigetfile('*.txt'); % 读取雷达数据
end
if FileName==0
    return;
end
data=importdata([PathName,'\\',FileName]);

[FileName1,PathName1,~] = uigetfile([PathName,'\\*.mat']); % 读取雷达数据
data1=importdata([PathName1,'\\',FileName1]);

i_star0=find(data.data(:,2)>0, 1 );
i_end=find(abs(diff(data.data(:,4)))>10,1);
data.data(i_end:end,:)=[];
i_star=find(diff(data.data(i_star0:end,2))>0,1)+i_star0;
i_ts0=find(data1.GPS(:,3)==data.data(i_star,2),1); 
ts0=data1.GPS(i_ts0,14)/1000;

ts=(data.data(i_star:end,2)+data.data(i_star:end,3))/1000;
ts=ts-ts(1)+ts0;
Re=6378137;
Lat=data.data(i_star:end,4);
Lng=data.data(i_star:end,5);
alt=data.data(i_star:end,6);
Vx=data.data(i_star:end,7);
Vy=data.data(i_star:end,8);
Vz=data.data(i_star:end,9);
V=(Vx.^2+Vy.^2).^0.5;

Xp=Re*Lng/57.3.*cosd(Lat(1));
Yp=Re*Lat/57.3;
Xp=Xp-Xp(1);
Yp=Yp-Yp(1);
% V=((Xp(2:end)-Xp(1:end-1)).^2+(Yp(2:end)-Yp(1:end-1)).^2).^0.5/0.02;
% Vz=(alt(2:end)-alt(1:end-1))/0.02;
% V=[0 ;V];
fai=atan2(data.data(i_star:end,8),data.data(i_star:end,7))*57.3;
temp=find(fai<0);
fai(temp)=fai(temp)+360;
% Vz=[0;Vz];
% c= [PathName,'\\','GPS1.mat'];
% save GPS1.mat ts Vx Vy Vz V
data_ck=[ts  data.data(i_star:end,4:end) Xp Yp V fai];
fid=fopen([PathName,'\\','GPS1.dat'],'w');
fprintf(fid,'t Lat纬度 Lng经度 高度 Vx Vy Vz NSats卫星数 RTK Xp Yp V fai\n');
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);      
  
    

