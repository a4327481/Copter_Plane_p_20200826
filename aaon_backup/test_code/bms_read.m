%% 从文本文件中导入数据
% 用于从文本文件中导入数据的脚本:
%
%   
%
% 由 MATLAB 于 2020-06-24 10:10:00 自动生成

%% 设置导入选项并导入数据
opts = delimitedTextImportOptions("NumVariables", 27);

% 指定范围和分隔符
opts.DataLines = [1, Inf];
opts.Delimiter = " ";

% 指定列名称和类型
opts.VariableNames = ["time_bms", "ASOC", "RSOC", "SOH", "Average_Time_To_empty", "Cell_Volt_0", "Cell_Volt_1", "Cell_Volt_2", "Cell_Volt_3", "Cell_Volt_4", "Cell_Volt_5", "Cycle_Count", "Design_Cap", "Design_Volt", "FCC", "FW_Version", "Manufacture_Date", "Other_Status", "Rem_Cap", "Run_Time_TO_Empty", "Safty_Status", "Serial_Num", "Temperature", "Total_Cycles", "Voltage", "Now_Current", "Average_Current"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% 指定文件级属性
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.ConsecutiveDelimitersRule = "join";
opts.LeadingDelimitersRule = "ignore";

% 导入数据
read_data = readtable("E:\飞行试验\V1000\新建文件夹\模式切换\2020-04-22 07-36-45.binV1000_bms.dat", opts);


%% 清除临时变量
clear opts
%%绘图
hold on
plot(read_data.time_bms,read_data.Average_Current)
plot(read_data.time_bms,read_data.Voltage)

