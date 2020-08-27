
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

fileID = fopen([PathName,'\\',FileName])
C = textscan(fileID,'%s');
fclose(fileID);
read.year=0;
read.month=0;
read.day=0;
read.hour=0;
read.min=0;
read.sec=0;
read.eflag=0;
read.nstar=0;
len=length(C{1,1})
i=1;
j=1;
header.a='>';
header.b='2019';
while(i<len)
    if (strcmp(C{1,1}{i},header.a)&&strcmp(C{1,1}{i+1},header.b))
        read.year(j,1)=str2num(C{1,1}{i+1});
        read.month(j,1)=str2num(C{1,1}{i+2});
        read.day(j,1)=str2num(C{1,1}{i+3});
        read.hour(j,1)=str2num(C{1,1}{i+4});
        read.min(j,1)=str2num(C{1,1}{i+5});
        read.sec(j,1)=str2double(C{1,1}{i+6});
        read.eflag(j,1)=str2num(C{1,1}{i+7});
        read.nstar(j,1)=str2num(C{1,1}{i+8});
        j=j+1;
        i=i+8;
    end
    i=i+1;       
end
C=[];
ts=read.sec+read.min*60+read.hour*3600+read.day*3600*24;
ts=ts-ts(1);
i_eflag=find(read.eflag==5);
j_eflag=find(read.eflag~=5);

data_ck=[read.year(i_eflag) read.month(i_eflag) read.day(i_eflag) read.hour(i_eflag) read.min(i_eflag) read.sec(i_eflag) read.eflag(i_eflag) read.nstar(i_eflag)];
fid=fopen([PathName,'\\',FileName,'mark.dat'],'w');
fprintf(fid,'read.year read.month read.day read.hour read.min read.sec read.eflag read.nstar\n');
    [count,~]=size(data_ck);
     for i=1:count
        fprintf(fid,'> %d %d %d %d %d %f  %d %d\n',data_ck(i,:));
    end
    fclose(fid);
data_ck=[ts(i_eflag)-ts(i_eflag(1)) read.year(i_eflag) read.month(i_eflag) read.day(i_eflag) read.hour(i_eflag) read.min(i_eflag) read.sec(i_eflag) read.eflag(i_eflag) read.nstar(i_eflag)];
fid=fopen([PathName,'\\',FileName,'markt.dat'],'w');
fprintf(fid,'t read.year read.month read.day read.hour read.min read.sec read.eflag read.nstar\n');
    [count,~]=size(data_ck);
     for i=1:count
        fprintf(fid,'%f %d %d %d %d %d %f  %d %d\n',data_ck(i,:));
    end
    fclose(fid);
    
data_ck=[ts(j_eflag) read.year(j_eflag) read.month(j_eflag) read.day(j_eflag) read.hour(j_eflag) read.min(j_eflag) read.sec(j_eflag) read.eflag(j_eflag) read.nstar(j_eflag)];
fid=fopen([PathName,'\\',FileName,'rinex.dat'],'w');
fprintf(fid,'t read.year read.month read.day read.hour read.min read.sec read.eflag read.nstar\n');
    [count,~]=size(data_ck);
     for i=1:count
        fprintf(fid,' %f %d %d %d %d %d %f  %d %d\n',data_ck(i,:));
    end
    fclose(fid);


%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
%     fclose(fid);
    
    hold on
%     plot(ts(j_eflag(2:end))-ts(j_eflag(1)),diff(ts(j_eflag)))
    plot(ts(i_eflag(1:end)),0.05*ones(length(i_eflag(1:end)),1),'o')
%     plot(ts(i_eflag(2:end-2))-ts(i_eflag(1)),diff(ts(i_eflag(1:end-2))),'b*')
    plot(ts(i_eflag(2:end-2)),diff(ts(i_eflag(1:end-2))),'b*')

 saveas(gcf,[PathName,'\\',FileName(1:end-6),'t.fig'],'fig') ;
 saveas(gcf,[PathName,'\\',FileName(1:end-6),'t.jpg'],'jpg') ;