%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mat 格式转换为画图
global PathName
if PathName~=0
    cd(PathName);
    [FileName,PathName,~] = uigetfile([PathName,'\\*.txt'],'MultiSelect', 'on'); % 读取雷达数据
else
    [FileName,PathName,~] = uigetfile('*.txt','MultiSelect', 'on'); % 读取雷达数据
end
if isnumeric(FileName)
    if FileName==0
        return;
    end
end
if ischar(FileName)
    nFile = 1;
elseif iscell(FileName)
    nFile = length(FileName);
end
clear newname
for i_file = 1:nFile
    if ischar(FileName)
        thisFileName = FileName;
    elseif iscell(FileName)
        thisFileName = FileName{i_file};
    end
    fprintf('正在处理第 %d/%d 个数据文件 "%s"\n',i_file,nFile,thisFileName);
    fileID = fopen([PathName,'\\',thisFileName]);
    C = fread(fileID);
    fclose(fileID);
    len=length(C);
    BLOCK_SIZE=9;
    m=floor(len/BLOCK_SIZE);
    ax=zeros(m,1);
    ay=zeros(m,1);
    az=zeros(m,1);
    i=1;
    j=1;
    while(i<(m-1)*BLOCK_SIZE)
        if(C(i)==hex2dec('AA')&&C(i+1)==hex2dec('FF'))
            ax(j)=double(typecast(uint8([C(i+2),C(i+3)]),'int16')')*0.01;
            ay(j)=double(typecast(uint8([C(i+4),C(i+5)]),'int16')')*0.01;
            az(j)=double(typecast(uint8([C(i+6),C(i+7)]),'int16')')*0.01;
            if((C(i+8)==mod(sum(C(i+2:i+7)),256)&&(abs(ax(j))<20)&&(abs(ay(j))<20)&&(abs(az(j))<20)))
                j=j+1;
                i=i+9;
            else
                i=i+1;
            end
            
        else
            i=i+1;
        end
        
    end
    t=((1:j-1)*0.001)';
    ax(j:end)=[];
    ay(j:end)=[];
    az(j:end)=[];
    data_ck=[t ax ay az];%数据数组
    newname{i_file} = strrep(thisFileName,'.','');
    newname{i_file} = ['data_',newname{i_file}];
    eval([newname{i_file},'= data_ck;']);
    fprintf('\t对应工作空间中的变量为\t%s\n',newname{i_file})
end
dataSaveFileName = 'datasave.mat';
delete(dataSaveFileName)
%%
fprintf('数据保存在%s\n',dataSaveFileName)
for i_file = 1:nFile
    if i_file == 1
        save(dataSaveFileName,newname{i_file})
    else
        save(dataSaveFileName,newname{i_file},'-append')
    end
end
% fid=fopen([PathName,'\\',FileName,'accel'],'w');
% fprintf(fid,'t ax ay az\n');
% fclose(fid);
% save([PathName,'\\',FileName,'accel'],'data_ck','-ascii','-append' )
%% 绘制时域图
fprintf('不同加速度数值时间轴未对齐\n')
plot_accel_timeDomain
%% 绘制频域图
plot_accel_freqDomain
%%
