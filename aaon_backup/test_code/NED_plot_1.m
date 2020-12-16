 t=695:0.01:702;


 
algo_curPosNED_0_o=interp1(time_sl_task_flight,algo_curPosNED_0,t);
algo_curPosNED_1_o=interp1(time_sl_task_flight,algo_curPosNED_1,t);

hold on
plot(algo_curPosNED_0_o,algo_curPosNED_1_o)






 