
hz_lowpass = 0.001;
for i_file = 1:nFile
    figure(211);
    thisVar = eval(newname{i_file});
    tempFs(i_file) = 1/mean(diff(thisTime));
    Fs = mean(tempFs); % 采样频率
    
    thisTime = thisVar(:,1);
    thisAccX = thisVar(:,2);
    thisAccY = thisVar(:,3);
    thisAccZ = thisVar(:,4);
    subplot(311)
    title('加速度PSD对比图')
    thisData = thisAccX;
    [pxx_x{i_file},f] = pwelch(thisData,[],[],[],Fs);
%     length(pxx_x{i_file})
    pmax = pwelch(thisData,[],[],[],Fs,'maxhold');
    pmin = pwelch(thisData,[],[],[],Fs,'minhold');
    plot(f/1000,pow2db(pxx_x{i_file}))
    hold on
    ylabel('Acc_x PSD (dB/Hz)')
    grid on
    %%
    subplot(312)
    thisData = thisAccY;
    [pxx_y{i_file},f] = pwelch(thisData,[],[],[],Fs);
    plot(f/1000,pow2db(pxx_y{i_file}))
    hold on
    xlabel('Frequency (kHz)')
    ylabel('Acc_y PSD (dB/Hz)')
    grid on
    %%
    subplot(313)
    thisData = thisAccZ;
    [pxx_z{i_file},f] = pwelch(thisData,[],[],[],Fs);
    plot(f/1000,pow2db(pxx_z{i_file}))
    hold on
    xlabel('Frequency (kHz)')
    ylabel('Acc_z PSD (dB/Hz)')
    
    grid on       
    figure(212);
    subplot(311)
    title('加速度PSD平滑对比图')
    plot(f/1000,lowpass(pow2db(pxx_x{i_file}),hz_lowpass,Fs))
    hold on    
    ylabel('Acc_x PSD (dB/Hz)')
    grid on
    subplot(312)
    plot(f/1000,lowpass(pow2db(pxx_y{i_file}),hz_lowpass,Fs))
    hold on    
    ylabel('Acc_y PSD (dB/Hz)')
    grid on    
    subplot(313)
    plot(f/1000,lowpass(pow2db(pxx_z{i_file}),hz_lowpass,Fs))
    hold on    
    ylabel('Acc_z PSD (dB/Hz)')
    grid on        
    xlabel('频率(Hz)')
end

legendStr = '';
if nFile > 1
    for i_file = 1:nFile
        legendStr = [legendStr,'''',FileName{i_file},'''',','];
    end
    legendStr(end) = [];
    subplot(311)
    eval(['legend(',legendStr,')']);
end

%% 增益
figure;
subplot(221)
plot(f,lowpass((pxx_x{4}),hz_lowpass,Fs)./lowpass((pxx_x{1}),hz_lowpass,Fs));
ylabel('acc_x增益 数据4与数据1比值');grid on;
subplot(222)
plot(f,lowpass((pxx_x{3}),hz_lowpass,Fs)./lowpass((pxx_x{1}),hz_lowpass,Fs));
ylabel('acc_x增益 数据3与数据1比值');grid on;
subplot(223)
plot(f,lowpass((pxx_x{4}),hz_lowpass,Fs)./lowpass((pxx_x{2}),hz_lowpass,Fs));
ylabel('acc_x增益 数据4与数据2比值');grid on;
xlabel('频率 (Hz)')
subplot(224)
plot(f,lowpass((pxx_x{3}),hz_lowpass,Fs)./lowpass((pxx_x{2}),hz_lowpass,Fs));
ylabel('acc_x增益 数据3与数据2比值');grid on;
xlabel('频率 (Hz)')

yspan = [-1,10];
figure;
subplot(221)
plot(f,lowpass((pxx_y{4}),hz_lowpass,Fs)./lowpass((pxx_y{1}),hz_lowpass,Fs));
ylabel('acc_y增益 数据4与数据1比值');grid on;
ylim(yspan)
subplot(222)
plot(f,lowpass((pxx_y{3}),hz_lowpass,Fs)./lowpass((pxx_y{1}),hz_lowpass,Fs));
ylabel('acc_y增益 数据3与数据1比值');grid on;
ylim(yspan)
subplot(223)
plot(f,lowpass((pxx_y{4}),hz_lowpass,Fs)./lowpass((pxx_y{2}),hz_lowpass,Fs));
ylabel('acc_y增益 数据4与数据2比值');grid on;
ylim(yspan)
xlabel('频率 (Hz)')
subplot(224)
plot(f,lowpass((pxx_y{3}),hz_lowpass,Fs)./lowpass((pxx_y{2}),hz_lowpass,Fs));
ylabel('acc_y增益 数据3与数据2比值');grid on;
ylim(yspan)
xlabel('频率 (Hz)')

figure;
subplot(221)
plot(f,lowpass((pxx_z{4}),hz_lowpass,Fs)./lowpass((pxx_z{1}),hz_lowpass,Fs));
ylabel('acc_z增益 数据4与数据1比值');
ylim(yspan)
subplot(222)
plot(f,lowpass((pxx_z{3}),hz_lowpass,Fs)./lowpass((pxx_z{1}),hz_lowpass,Fs));
ylabel('acc_z增益 数据3与数据1比值');
ylim(yspan)
subplot(223)
plot(f,lowpass((pxx_z{4}),hz_lowpass,Fs)./lowpass((pxx_z{2}),hz_lowpass,Fs));
ylabel('acc_z增益 数据4与数据2比值');
ylim(yspan)
xlabel('频率 (Hz)')
subplot(224)
plot(f,lowpass((pxx_z{3}),hz_lowpass,Fs)./lowpass((pxx_z{2}),hz_lowpass,Fs));
ylabel('acc_z增益 数据3与数据2比值');
ylim(yspan)
xlabel('频率 (Hz)')