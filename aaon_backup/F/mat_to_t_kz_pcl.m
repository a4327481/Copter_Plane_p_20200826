% function  mat_to_matlab(varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%mat 格式转换为画图
global PathName
if PathName~=0
    cd(PathName);
    [FileName,PathName,~] = uigetfile([PathName,'\\*.*']); %  
else
    [FileName,PathName,~] = uigetfile('*.*'); % 
end
if FileName==0
    return;
end
finf = dir([PathName,'\\*.mat'])
n = length(finf);
cell_name={finf.name}
nat_name=natsort(cell_name)
%%%%%%%%%%%%%%%%%%%%%%%
a=100;
b_imu=500;
% t = cell(n,1);             %%生成n*1的元胞
%%%%%%%%%%%%%%%%%%%%%%%%%%%
i=1;
for j=1:n
    filename = [PathName,'\\',nat_name{j}];  %%构造第k个文件的位置（合并文件路径和文件名）
    load(filename);%%将文件夹中的文件每个作为一个元胞按行存储在元胞中
    if ~(exist('IMU','var'))
        continue
    end
    fid1=fopen(filename(1:end-4),'r');
    ID=fread(fid1,20,'uint8');
    ID1=char(ID(4:end)');
    fclose(fid1);
        
    len_imu=length(IMU);
    if (len_imu>10000)
    t(i).imu=IMU(a+b_imu:end,2)/1000;
    t(i).imu2=IMU2(a+b_imu:end,2)/1000;
    t(i).att=ATT(a:end,2)/1000;
    t(i).arsp=ARSP(a:end,2)/1000;
    t(i).baro=BARO(a:end,2)/1000;
    t(i).ctun=CTUN(a:end,2)/1000;
    t(i).gps=GPS(a:end,14)/1000;
    t(i).sonar=SONAR(a:end,2)/1000;    
%     t(i).rcout=RCOUT(a:end,2)/1000; 
    t(i).name=nat_name{j};
    t(i).ID=ID1;
    i=i+1;
    end 
    clear IMU        
end
len_t=i-1;
for i=1:len_t
    t(i).dimu=diff(t(i).imu);
    temp_a=find(abs((t(i).dimu))<10 );
    t(i).dimu_max=max(t(i).dimu(temp_a));
   
end
% hold on
% for i=1:len_t
%     t(i).dgps=mean(diff(t(i).gps));  
%     plot(t(i).imu(2:end),diff(t(i).imu),'-o')
% end 
%     
figure
hold on
% plot([t.dgps]*5)
plot([t.dimu_max])
 saveas(gcf,[PathName,'\\',FileName(1:end-6),'t.fig'],'fig') ;
 saveas(gcf,[PathName,'\\',FileName(1:end-6),'t.jpg'],'jpg') ;
%    
% figure
% hold on
% grid on;
% plot(t.imu(2:end),diff(t.imu),'-o')
% plot(t.imu2(2:end),diff(t.imu2),'-o')
% plot(t.att(2:end),diff(t.att),'-o')
% plot(t.arsp(2:end),diff(t.arsp),'-o')
% plot(t.baro(2:end),diff(t.baro),'-o')
% plot(t.ctun(2:end),diff(t.ctun),'-o')
% plot(t.gps(2:end),diff(t.gps),'-o')
% plot(t.sonar(2:end),diff(t.sonar),'-o')
% plot(t.rcout(2:end),diff(t.rcout),'-o')
% legend('imu','imu2','att','arsp','baro','ctun','gps','sonar','rcout');
% %  saveas(gcf,[PathName,'\\','t.dat'],'fig') ;
% title('解算周期统计/diff(t)')
% xlabel('t(s)')
% ylabel('dt(s)')
%  saveas(gcf,[PathName,'\\',FileName(1:end-6),'t.fig'],'fig') ;
%  saveas(gcf,[PathName,'\\',FileName(1:end-6),'t.jpg'],'jpg') ;

