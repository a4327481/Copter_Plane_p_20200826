figure;
plot(algo_curPosNED_1,algo_curPosNED_0);hold on;grid on;
for i = 1:100:length(time_sl_task_flight)
    plot(algo_curPosNED_1(i),algo_curPosNED_0(i),'ro');hold on;
    text(algo_curPosNED_1(i),algo_curPosNED_0(i),num2str(time_sl_task_flight(i)));hold on; 
end