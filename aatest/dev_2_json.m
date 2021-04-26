%% 从文本文件中导入数据
% 用于从以下文本文件中导入数据的脚本:
%
%    filename: E:\matlab_code\Copter_Plane_p_20200826\test\data_dev.m
%
% 由 MATLAB 于 2020-12-10 16:14:12 自动生成

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 4);

% 指定范围和分隔符
opts.DataLines = [1, Inf];
opts.Delimiter = ["%", ";", "="];

% 指定列名称和类型
opts.VariableNames = ["fullname", "data", "name",'Var'];
opts.VariableTypes = ["char", "double", "char","char"];

% 指定文件级属性
opts.ImportErrorRule = "error";
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.ConsecutiveDelimitersRule = "join";

% 指定变量属性
opts = setvaropts(opts, ["name", "Var"], "WhitespaceRule", "preserve");
opts = setvaropts(opts, ["name", "Var"], "EmptyFieldRule", "auto");

% 导入数据
datadev1 = readtable("D:\matlab_code\Copter_Plane_p_20200826\aatest\data_devg2.m", opts);

%% 转换为输出类型
datadev1 = table2cell(datadev1);
numIdx = cellfun(@(x) ~isnan(str2double(x)), datadev1);
datadev1(numIdx) = cellfun(@(x) {str2double(x)}, datadev1(numIdx));

%% 清除临时变量
clear opts
len=size(datadev1,1);
out=[];
for i=1:len
%     out.ParamDriver{1, i}.index     =i;
%     out.ParamDriver{1, i}.nameFull  =datadev1{i, 1}  ;
    out.ParamDriver{1, i}.n      =datadev1{i, 3}  ;
    out.ParamDriver{1, i}.v     =datadev1{i, 2}  ;
end
savejson('',out,'ParamDriver.json');
out=[];
for i=1:len
    out.ParamDriver{1, i}.index     =i;
    out.ParamDriver{1, i}.nameFull  =datadev1{i, 1}  ;
    out.ParamDriver{1, i}.name      =datadev1{i, 3}  ;
%     out.ParamDriver{1, i}.value     =datadev1{i, 2}  ;
end
savejson('',out,'ParamDriver_Full.json');
