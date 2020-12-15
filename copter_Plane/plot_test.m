t=1000:1150;
global HD
global LOCATION_SCALING_FACTOR
len=length(t);
 LOCATION_SCALING_FACTOR = 0.011131884502145034;
 HD=180/pi;

algo_prePathPoint_LLA_0_o=interp1(time_sl_taskmode,algo_prePathPoint_LLA_0,t);
algo_prePathPoint_LLA_1_o=interp1(time_sl_taskmode,algo_prePathPoint_LLA_1,t);

algo_turnCenterLL_0_o=interp1(time_sl_taskmode,algo_turnCenterLL_0,t);
algo_turnCenterLL_1_o=interp1(time_sl_taskmode,algo_turnCenterLL_1,t);

algo_curPathPoint_LLA_0_o=interp1(time_sl_taskmode,algo_curPathPoint_LLA_0,t);
algo_curPathPoint_LLA_1_o=interp1(time_sl_taskmode,algo_curPathPoint_LLA_1,t);

algo_curLLA_0_o=interp1(time_sl_taskmode,algo_curLLA_0,t);
algo_curLLA_1_o=interp1(time_sl_taskmode,algo_curLLA_1,t);

algo_curLLA_o=[algo_curLLA_0_o;algo_curLLA_1_o]'*1e7;
algo_curPathPoint_LLA_o=[algo_curPathPoint_LLA_0_o;algo_curPathPoint_LLA_1_o]'*1e7;
algo_turnCenterLL_o=[algo_turnCenterLL_0_o;algo_turnCenterLL_1_o]'*1e7;
algo_prePathPoint_LLA_o=[algo_prePathPoint_LLA_0_o;algo_prePathPoint_LLA_1_o]'*1e7;

algo_curPathPoint_LLA_o_NE=zeros(len,2);
algo_curLLA_o_NE=zeros(len,2);
algo_prePathPoint_LLA_o_NE=zeros(len,2);

for i=1:len
algo_curPathPoint_LLA_o_NE(i,:)=get_distance_NE( algo_curPathPoint_LLA_o(i,:),algo_curLLA_o(1,:)); 
algo_curLLA_o_NE(i,:)=get_distance_NE( algo_curLLA_o(i,:),algo_curLLA_o(1,:)); 
algo_prePathPoint_LLA_o_NE(i,:)=get_distance_NE( algo_prePathPoint_LLA_o(i,:),algo_curLLA_o(1,:)); 
end

hold on
plot(algo_curPathPoint_LLA_o_NE(:,1),algo_curPathPoint_LLA_o_NE(:,2),'*')
% plot(algo_turnCenterLL_0_o,algo_turnCenterLL_1_o,'o')
plot(algo_curLLA_o_NE(:,1),algo_curLLA_o_NE(:,2))
plot(algo_prePathPoint_LLA_o_NE(:,1),algo_prePathPoint_LLA_o_NE(:,2),'o')


% hold on
% plot(algo_prePathPoint_LLA_0_o,algo_prePathPoint_LLA_1_o,'*')
% % plot(algo_turnCenterLL_0_o,algo_turnCenterLL_1_o,'o')
% plot(algo_curPathPoint_LLA_0_o,algo_curPathPoint_LLA_1_o,'o')
% plot(algo_curLLA_0_o,algo_curLLA_1_o)


 