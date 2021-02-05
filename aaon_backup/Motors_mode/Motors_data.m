%m电机模型分析
%% 导入电子表格中的数据
% 用于从以下电子表格导入数据的脚本:
%
%    工作簿: D:\新飞控\V10\705电机44.4V下快推慢回.xls
%    工作表: 705电机44.4V下快推慢回
%
% 由 MATLAB 于 2021-02-03 10:48:53 自动生成

%% Set up the Import Options and import the data
opts = spreadsheetImportOptions("NumVariables", 16);

% 指定工作表和范围
% opts.Sheet = "705电机44.4V下快推慢回";
% opts.DataRange = "A2:P726";

% 指定列名称和类型
opts.VariableNames = ["number", "time", "thr", "PWM", "V", "A", "Gf", "Nm", "RPM", "W", "W1", "motor_xiaolv", "GfW", "GfW1", "mAh", "Wh"];
opts.VariableTypes = ["double", "datetime", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% 指定变量属性
opts = setvaropts(opts, "time", "InputFormat", "");

% 导入数据
Motor_data0 = readtable("D:\新飞控\V10\705电机44.4V下快推慢回.xls", opts, "UseExcel", false);
Motor_data0=Motor_data0(62:end,:);
Motor_data = readtable("D:\新飞控\V10\705电机44.4V下不同油门推力.xls", opts, "UseExcel", false);
Motor_data(1,:)=[];
 opts.Sheet = "今天第二次";
Motor_data1 = readtable("D:\新飞控\V10\705电机44.4V下快推慢回(1).xls", opts, "UseExcel", false);
Motor_data1(1:21,:)=[];
 opts.Sheet = "今天第三次";
Motor_data2 = readtable("D:\新飞控\V10\705电机44.4V下快推慢回(1).xls", opts, "UseExcel", false);
Motor_data2(1:17,:)=[];

% Motor_data1=Motor_data1(62:end,:);
%数据处理 data0
Motor_data0.time=[];
Motor_data0o=Motor_data0;
PWMu=unique(Motor_data0.PWM);
len=length(PWMu);
Motor_data0o(len+1:end,:)=[];
for i=1:len
    Motor_data0o{i,:}=mean(Motor_data0{Motor_data0.PWM==PWMu(i),:},1);
    if(PWMu(i)==1635)
        a=1;
    end
end
Motor_data0o.PWM=PWMu;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%数据处理 data
Motor_data.time=[];
Motor_datao=Motor_data;
PWMu=unique(Motor_data.PWM);
len=length(PWMu);
Motor_datao(len+1:end,:)=[];
for i=1:len
    Motor_datao{i,:}=mean(Motor_data{Motor_data.PWM==PWMu(i),:},1);
end
Motor_datao.PWM=PWMu;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%数据处理 data
Motor_data1.time=[];
Motor_data1o=Motor_data1;
PWMu=unique(Motor_data1.PWM);
len=length(PWMu);
Motor_data1o(len+1:end,:)=[];
for i=1:len
    Motor_data1o{i,:}=mean(Motor_data1{Motor_data1.PWM==PWMu(i),:},1);
end
Motor_data1o.PWM=PWMu;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%数据处理 data
Motor_data2.time=[];
Motor_data2o=Motor_data2;
PWMu=unique(Motor_data2.PWM);
len=length(PWMu);
Motor_data2o(len+1:end,:)=[];
for i=1:len
    Motor_data2o{i,:}=mean(Motor_data2{Motor_data2.PWM==PWMu(i),:},1);
end
Motor_data2o.PWM=PWMu;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% hold on
%  plot(Motor_data0o.PWM,-Motor_data0o.Gf)
%  plot(Motor_data1o.PWM,-Motor_data1o.Gf)
%  plot(Motor_data2o.PWM,-Motor_data2o.Gf)
% hold on
%  plot(Motor_data0o.PWM,-Motor_data0o.RPM)
%  plot(Motor_data1o.PWM,-Motor_data1o.RPM)
%  plot(Motor_data2o.PWM,-Motor_data2o.RPM)
%  hold on
%  plot(Motor_data0o.RPM,-Motor_data0o.Gf)
%  plot(Motor_data1o.RPM,-Motor_data1o.Gf)
%  plot(Motor_data2o.RPM,-Motor_data2o.Gf)
 
% % plot(Motor_data0o.PWM,-Motor_data0o.Gf)
% % plot(Motor_data0.PWM,-Motor_data0.Gf)
% plot(Motor_data0o.PWM(32:end),Motor_data0o.RPM(32:end))
% 
% % plot(Rotor_2608.w,Rotor_2608.F)
% % plot(Rotor_2608_f.w,Rotor_2608_f.F)
% plot(Motor_data0o.RPM*2*pi/60,-Motor_data0o.Gf*9.8/1000*1.25)
% plot(Motor_data0o.RPM*2*pi/60,-Motor_data0o.Gf*9.8/1000)
% % plot(Motor_datao.RPM*2*pi/60,-Motor_datao.Gf*9.8/1000)
% 
% plot(Motor_data0o.RPM.^2/1000/2/1.18,-Motor_data0o.Gf*9.8/1000)
% 
% plot(Motor_data0.PWM,Motor_data0.RPM)
% plot(Motor_data0.PWM,Motor_data0.RPM.^2/1000/2/1.18)
% plot(Motor_data.PWM,Motor_data.Gf)
% 
hold on
plot((-Motor_data0o.Gf(24:end)-(-Motor_data0o.Gf(24)))/(-Motor_data0o.Gf(end)-(-Motor_data0o.Gf(24))),(Motor_data0o.RPM(24:end)-Motor_data0o.RPM(24))/(Motor_data0o.RPM(end)-Motor_data0o.RPM(24)))
% plot(-Motor_data1o.Gf(1:end)/-Motor_data1o.Gf(end),Motor_data1o.RPM(1:end)/Motor_data1o.RPM(end))
% plot(-Motor_data2o.Gf(1:end)/-Motor_data2o.Gf(end),Motor_data2o.RPM(1:end)/Motor_data2o.RPM(end))

hold on
% plot(-Motor_data0o.Gf(24:end)/-Motor_data0o.Gf(end),Motor_data0o.thr(24:end)/100)
% plot(-Motor_data1o.Gf(1:end)/-Motor_data1o.Gf(end),Motor_data1o.thr(1:end)/100)
% plot(-Motor_data2o.Gf(1:end)/-Motor_data2o.Gf(end),Motor_data2o.thr(1:end)/100)
% hold on
% plot(-Motor_data0o.RPM(1:end)/-Motor_data0o.RPM(end),Motor_data0o.thr(1:end)/100)
% plot(-Motor_data1o.RPM(1:end)/-Motor_data1o.RPM(end),Motor_data1o.thr(1:end)/100)
% plot(-Motor_data2o.RPM(1:end)/-Motor_data2o.RPM(end),Motor_data2o.thr(1:end)/100)
%% 清除临时变量
clear opts