figure;
% % 测量数据时间同步
% for i_file = 1:nFile
%     thisVar = eval(newname{i_file});
%     lenData(i_file) = length(thisVar);
% end
% [~,idxMax] = max(lenData);
% t21 = finddelay(eval(newname{1}(:,2)),eval(newname{2})(:,2))
% aa = data_1TXT(:,2);
% bb = data_2TXT(:,2);
% [aa_align,bb_align] = alignsignals(aa,bb);
% figure;
% subplot(211)
% plot(aa);hold on;
% plot(bb);hold on;
% subplot(212)
% plot(aa_align);hold on;
% plot(bb_align);hold on;
% 绘制数据
for i_file = 1:nFile
    thisVar = eval(newname{i_file});
    thisTime = thisVar(:,1);
    thisAccX = thisVar(:,2);
    thisAccY = thisVar(:,3);
    thisAccZ = thisVar(:,4);
    subplot(311)
    title('加速度值比图')
    plot(thisTime,thisAccX); hold on;
    grid on;
    ylabel('accel_x (m/s^2)');
    subplot(312)
    plot(thisTime,thisAccX); hold on;
    grid on;
    ylabel('accel_y (m/s^2)');
    subplot(313)
    plot(thisTime,thisAccX); hold on;
    grid on;
    ylabel('accel_z (m/s^2)');
    xlabel('time (sec)')
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