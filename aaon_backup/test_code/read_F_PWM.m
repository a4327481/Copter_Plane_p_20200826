%% 导入电子表格中的数据
% 用于从以下电子表格导入数据的脚本:
%
%    工作簿: D:\新飞控\V10\悬停电机\悬停电机\悬停电机1000-1900（约700ms）\20201214-110159-变速测试-任务名称.xlsx
%    工作表: 20201214-110159-变速测试-任务名称
%
% 由 MATLAB 于 2020-12-15 15:36:47 自动生成

%% Set up the Import Options and import the data
opts = spreadsheetImportOptions("NumVariables", 14);

% 指定工作表和范围
% opts.Sheet = "20201214-110159-变速测试-任务名称";
opts.DataRange = "A3:N2021";

% 指定列名称和类型
opts.VariableNames = ["ms", "VarName2", "VarName3", "VarName4", "F", "VarName6", "VarName7", "EMF", "VarName9", "PWM", "VarName11", "VarName12", "VarName13", "VarName14"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% 导入数据
 read_data = readtable("D:\新飞控\V10\悬停电机\悬停电机\悬停电机1000-1900（约700ms）\20201214-110159-变速测试-任务名称.xlsx", opts, "UseExcel", false);
% read_data = readtable("D:\新飞控\V10\悬停电机\悬停电机\悬停电机1050-1950（约350ms）\20201214-110815-变速测试-任务名称.xlsx", opts, "UseExcel", false);

%% 清除临时变量
clear opts
read_data1=read_data;
read_data1.s=(read_data1.ms-read_data1.ms(1))/1000;
plot(read_data1.s,read_data1.F/18000*1000)
hold on
plot(read_data1.s+0.1,read_data1.PWM-1000)
