%% 导入电子表格中的数据
% 用于从以下电子表格导入数据的脚本:
%
%    工作簿: D:\新飞控\V1000\V1000电机油门及拉力对应表(1).xls
%    工作表: 航源5208电机1758正桨21.6V满油门
%
% 由 MATLAB 于 2021-03-22 15:11:13 自动生成

%% Set up the Import Options and import the data
opts = spreadsheetImportOptions("NumVariables", 17);
% 指定列名称和类型
opts.VariableNames = ["VarName1", "VarName2", "thr", "V", "A", "VarName6", "VarName7", "VarName8", "VarName9", "VarName10", "VarName11", "VarName12", "VarName13", "VarName14", "VarName15", "VarName16", "VarName17"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% 指定工作表和范围
opts.Sheet = "航源5208电机1758正桨21.6V满油门";
opts.DataRange = "A3:Q183";

% 导入数据
V10001S0 = readtable("D:\新飞控\V1000\V1000电机油门及拉力对应表(1).xls", opts, "UseExcel", false);

% 指定工作表和范围
opts.Sheet = "航源5208电机1758反桨21.6V满油门";
opts.DataRange = "A3:Q162";

% 导入数据
V10001S1 = readtable("D:\新飞控\V1000\V1000电机油门及拉力对应表(1).xls", opts, "UseExcel", false);

% 指定工作表和范围
opts.Sheet = "航源3515电机11X8正桨21.6V满油门";
opts.DataRange = "A3:Q170";

% 导入数据
V10001S2 = readtable("D:\新飞控\V1000\V1000电机油门及拉力对应表(1).xls", opts, "UseExcel", false);

opts.Sheet = "航源3515电机11X8反桨21.6V满油门";
opts.DataRange = "A3:Q184";

% 导入数据
V10001S3 = readtable("D:\新飞控\V1000\V1000电机油门及拉力对应表(1).xls", opts, "UseExcel", false);


%% 清除临时变量
clear opts
plot(V10001S0.thr,V10001S0.A,V10001S1.thr,V10001S1.A,V10001S2.thr,V10001S2.A,V10001S3.thr,V10001S3.A)
len=length(V10001S0.thr);
V10001S0o=zeros(100,2);
V10001S1o=zeros(100,2);
V10001S2o=zeros(100,2);
V10001S3o=zeros(100,2);
j=1;
for i=1:100
    temp=find(V10001S0.thr==i);
    if(~isempty(temp))
        V10001S0o(j,1)=i;
        V10001S0o(j,2)=mean(V10001S0.A(temp));
        j=j+1;
    end
end
V10001S0o(j:end,:)=[];

j=1;
for i=1:100
    temp=find(V10001S1.thr==i);
    if(~isempty(temp))
        V10001S1o(j,1)=i;
        V10001S1o(j,2)=mean(V10001S1.A(temp));
        j=j+1;
    end
end
V10001S1o(j:end,:)=[];

j=1;
for i=1:100
    temp=find(V10001S2.thr==i);
    if(~isempty(temp))
        V10001S2o(j,1)=i;
        V10001S2o(j,2)=mean(V10001S2.A(temp));
        j=j+1;
    end
end
V10001S2o(j:end,:)=[];

j=1;
for i=1:100
    temp=find(V10001S3.thr==i);
    if(~isempty(temp))
        V10001S3o(j,1)=i;
        V10001S3o(j,2)=mean(V10001S3.A(temp));
        j=j+1;
    end
end
V10001S3o(j:end,:)=[];


plot(V10001S0.thr,V10001S0.A,V10001S0o(:,1),V10001S0o(:,2))
save thr_Ao.mat V10001S0o V10001S1o V10001S2o V10001S3o 
