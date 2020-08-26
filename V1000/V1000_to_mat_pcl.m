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
finf = dir([PathName,'\\*.bin']);
n = length(finf);
cell_name={finf.name};
nat_name=natsort(cell_name);
%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j=1:n
    filename = [PathName,'\\',nat_name{j}];  %%构造第k个文件的位置（合并文件路径和文件名）
    fp=fopen(filename,'r');
    data = fread(fp);
    fclose(fp);   
    V1000_to_mat_load();
end



