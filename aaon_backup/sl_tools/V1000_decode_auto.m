baseIMUtime = IN_SENSOR.IMU1.time;
baseIMUtime = IN_SENSOR.IMU1.time;
% /* +===========+================================+=============+============+==============+ */
% /* +===========+================================+=============+============+==============+ */
temp = reshape([data(find(mod(Count,1)==0),1:2)'],1,[]);
save_count=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.save_count = save_count; % create struct
temp = reshape([data(find(mod(Count,1)==0),251:254)'],1,[]);
save_time=double(typecast(uint8(temp),'uint32')')/1*1.0000000000;
TempName.save_time = save_time; % create struct
temp = reshape([data(find(mod(Count,8)==0),255:256)'],1,[]);
FCVERSION=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.FCVERSION = FCVERSION; % create struct
temp = reshape([data(find(mod(Count,8)==1),255:256)'],1,[]);
GPSVERSION=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.GPSVERSION = GPSVERSION; % create struct
temp = reshape([data(find(mod(Count,8)==2),255:256)'],1,[]);
LOADERVERSION=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.LOADERVERSION = LOADERVERSION; % create struct
temp = reshape([data(find(mod(Count,8)==3),255:256)'],1,[]);
MCU2VERSION=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.MCU2VERSION = MCU2VERSION; % create struct
temp = reshape([data(find(mod(Count,8)==4),255:256)'],1,[]);
ALGO_LOGICVERSION=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.ALGO_LOGICVERSION = ALGO_LOGICVERSION; % create struct
temp = reshape([data(find(mod(Count,8)==5),255:256)'],1,[]);
ALGO_DRIVERVERSION=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.ALGO_DRIVERVERSION = ALGO_DRIVERVERSION; % create struct
temp = reshape([data(find(mod(Count,4)==3),121:124)'],1,[]);
task_1ms_total_cnt=double(typecast(uint8(temp),'uint32')')/1*1.0000000000;
TempName.task_1ms_total_cnt = task_1ms_total_cnt; % create struct
temp = reshape([data(find(mod(Count,4)==3),125:128)'],1,[]);
task_4ms_total_cnt=double(typecast(uint8(temp),'uint32')')/1*1.0000000000;
TempName.task_4ms_total_cnt = task_4ms_total_cnt; % create struct
temp = reshape([data(find(mod(Count,4)==3),129:132)'],1,[]);
task_12ms_total_cnt=double(typecast(uint8(temp),'uint32')')/1*1.0000000000;
TempName.task_12ms_total_cnt = task_12ms_total_cnt; % create struct
temp = reshape([data(find(mod(Count,4)==3),133:136)'],1,[]);
task_100ms_total_cnt=double(typecast(uint8(temp),'uint32')')/1*1.0000000000;
TempName.task_100ms_total_cnt = task_100ms_total_cnt; % create struct
temp = reshape([data(find(mod(Count,8)==0),207:208)'],1,[]);
lose_connectgs_cntms=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.lose_connectgs_cntms = lose_connectgs_cntms; % create struct
temp = reshape([data(find(mod(Count,4)==3),265:266)'],1,[]);
GPS_week_num=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.GPS_week_num = GPS_week_num; % create struct
temp = reshape([data(find(mod(Count,4)==3),267:270)'],1,[]);
GPS_time_ms=double(typecast(uint8(temp),'uint32')')/1*1.0000000000;
TempName.GPS_time_ms = GPS_time_ms; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,1)==0),3:6)'],1,[]);
ax=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.ax = ax; % create struct
temp = reshape([data(find(mod(Count,1)==0),7:10)'],1,[]);
ay=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.ay = ay; % create struct
temp = reshape([data(find(mod(Count,1)==0),11:14)'],1,[]);
az=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.az = az; % create struct
temp = reshape([data(find(mod(Count,1)==0),15:18)'],1,[]);
gx=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.gx = gx; % create struct
temp = reshape([data(find(mod(Count,1)==0),19:22)'],1,[]);
gy=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.gy = gy; % create struct
temp = reshape([data(find(mod(Count,1)==0),23:26)'],1,[]);
gz=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.gz = gz; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,4)==1),239:242)'],1,[]);
a2x=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.a2x = a2x; % create struct
temp = reshape([data(find(mod(Count,4)==1),257:260)'],1,[]);
a2y=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.a2y = a2y; % create struct
temp = reshape([data(find(mod(Count,4)==1),261:264)'],1,[]);
a2z=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.a2z = a2z; % create struct
temp = reshape([data(find(mod(Count,4)==1),265:268)'],1,[]);
g2x=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.g2x = g2x; % create struct
temp = reshape([data(find(mod(Count,4)==1),269:272)'],1,[]);
g2y=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.g2y = g2y; % create struct
temp = reshape([data(find(mod(Count,4)==1),273:276)'],1,[]);
g2z=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.g2z = g2z; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,4)==2),239:242)'],1,[]);
a3x=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.a3x = a3x; % create struct
temp = reshape([data(find(mod(Count,4)==2),257:260)'],1,[]);
a3y=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.a3y = a3y; % create struct
temp = reshape([data(find(mod(Count,4)==2),261:264)'],1,[]);
a3z=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.a3z = a3z; % create struct
temp = reshape([data(find(mod(Count,4)==2),265:268)'],1,[]);
g3x=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.g3x = g3x; % create struct
temp = reshape([data(find(mod(Count,4)==2),269:272)'],1,[]);
g3y=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.g3y = g3y; % create struct
temp = reshape([data(find(mod(Count,4)==2),273:276)'],1,[]);
g3z=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.g3z = g3z; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,2)==1),41:42)'],1,[]);
altitue=double(typecast(uint8(temp),'int16')')/32768*2000.0000000000;
TempName.altitue = altitue; % create struct
temp = reshape([data(find(mod(Count,2)==1),43:44)'],1,[]);
temperature=double(typecast(uint8(temp),'int16')')/32768*100.0000000000;
TempName.temperature = temperature; % create struct
temp = reshape([data(find(mod(Count,2)==1),45:46)'],1,[]);
pressure=double(typecast(uint8(temp),'int16')')/32768*1100.0000000000;
TempName.pressure = pressure; % create struct
temp = reshape([data(find(mod(Count,2)==1),47:48)'],1,[]);
temperature_gs=double(typecast(uint8(temp),'int16')')/32768*1100.0000000000;
TempName.temperature_gs = temperature_gs; % create struct
temp = reshape([data(find(mod(Count,2)==1),49:50)'],1,[]);
pressure_gs=double(typecast(uint8(temp),'int16')')/32768*1100.0000000000;
TempName.pressure_gs = pressure_gs; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,4)==0),109:110)'],1,[]);
x=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.x = x; % create struct
temp = reshape([data(find(mod(Count,4)==0),111:112)'],1,[]);
y=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.y = y; % create struct
temp = reshape([data(find(mod(Count,4)==0),113:114)'],1,[]);
z=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.z = z; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,4)==0),115:116)'],1,[]);
x=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.x = x; % create struct
temp = reshape([data(find(mod(Count,4)==0),117:118)'],1,[]);
y=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.y = y; % create struct
temp = reshape([data(find(mod(Count,4)==0),119:120)'],1,[]);
z=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.z = z; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,4)==0),121:122)'],1,[]);
x=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.x = x; % create struct
temp = reshape([data(find(mod(Count,4)==0),123:124)'],1,[]);
y=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.y = y; % create struct
temp = reshape([data(find(mod(Count,4)==0),125:126)'],1,[]);
z=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.z = z; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,2)==1),167:168)'],1,[]);
x=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.x = x; % create struct
temp = reshape([data(find(mod(Count,2)==1),169:170)'],1,[]);
y=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.y = y; % create struct
temp = reshape([data(find(mod(Count,2)==1),171:172)'],1,[]);
z=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.z = z; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,2)==1),191:192)'],1,[]);
x=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.x = x; % create struct
temp = reshape([data(find(mod(Count,2)==1),193:194)'],1,[]);
y=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.y = y; % create struct
temp = reshape([data(find(mod(Count,2)==1),195:196)'],1,[]);
z=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.z = z; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,4)==3),117:117)'],1,[]);
SNR=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.SNR = SNR; % create struct
temp = reshape([data(find(mod(Count,4)==3),118:118)'],1,[]);
Flag=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.Flag = Flag; % create struct
temp = reshape([data(find(mod(Count,4)==3),119:120)'],1,[]);
Range=double(typecast(uint8(temp),'int16')')/32768*0.0100000000;
TempName.Range = Range; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,4)==1),109:112)'],1,[]);
iTOW=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.iTOW = iTOW; % create struct
temp = reshape([data(find(mod(Count,4)==1),113:116)'],1,[]);
velN=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.velN = velN; % create struct
temp = reshape([data(find(mod(Count,4)==1),117:120)'],1,[]);
velE=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.velE = velE; % create struct
temp = reshape([data(find(mod(Count,4)==1),121:124)'],1,[]);
velD=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.velD = velD; % create struct
temp = reshape([data(find(mod(Count,4)==1),125:128)'],1,[]);
lon=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.lon = lon; % create struct
temp = reshape([data(find(mod(Count,4)==1),129:132)'],1,[]);
lat=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.lat = lat; % create struct
temp = reshape([data(find(mod(Count,4)==1),133:136)'],1,[]);
height=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.height = height; % create struct
temp = reshape([data(find(mod(Count,16)==13),243:246)'],1,[]);
hMSL=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.hMSL = hMSL; % create struct
temp = reshape([data(find(mod(Count,4)==0),127:128)'],1,[]);
pDOP=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.pDOP = pDOP; % create struct
temp = reshape([data(find(mod(Count,4)==0),129:129)'],1,[]);
numSV=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.numSV = numSV; % create struct
temp = reshape([data(find(mod(Count,4)==0),131:134)'],1,[]);
hAcc=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.hAcc = hAcc; % create struct
temp = reshape([data(find(mod(Count,8)==1),197:200)'],1,[]);
vAcc=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.vAcc = vAcc; % create struct
temp = reshape([data(find(mod(Count,8)==1),201:204)'],1,[]);
headAcc=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.headAcc = headAcc; % create struct
temp = reshape([data(find(mod(Count,8)==1),205:208)'],1,[]);
sAcc=double(typecast(uint8(temp),'single')')/1*1.0000000000;
TempName.sAcc = sAcc; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,8)==0),205:205)'],1,[]);
GSConnected=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.GSConnected = GSConnected; % create struct
temp = reshape([data(find(mod(Count,2)==1),187:187)'],1,[]);
Remote_Disconnect_Status=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.Remote_Disconnect_Status = Remote_Disconnect_Status; % create struct
temp = reshape([data(find(mod(Count,4)==0),135:136)'],1,[]);
remote_time_cnt=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.remote_time_cnt = remote_time_cnt; % create struct
temp = reshape([data(find(mod(Count,2)==1),188:188)'],1,[]);
V1000_SwitchMode=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.V1000_SwitchMode = V1000_SwitchMode; % create struct
temp = reshape([data(find(mod(Count,2)==0),41:41)'],1,[]);
Switch_LockStatus=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.Switch_LockStatus = Switch_LockStatus; % create struct
temp = reshape([data(find(mod(Count,2)==0),42:42)'],1,[]);
Lock_Status=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.Lock_Status = Lock_Status; % create struct
temp = reshape([data(find(mod(Count,2)==0),43:43)'],1,[]);
rtM_ddrowkmode=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.rtM_ddrowkmode = rtM_ddrowkmode; % create struct
temp = reshape([data(find(mod(Count,2)==0),44:44)'],1,[]);
PLANE_COPTER_MODE=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.PLANE_COPTER_MODE = PLANE_COPTER_MODE; % create struct
temp = reshape([data(find(mod(Count,2)==0),45:46)'],1,[]);
remote_ct_srctilt=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.remote_ct_srctilt = remote_ct_srctilt; % create struct
temp = reshape([data(find(mod(Count,2)==0),47:48)'],1,[]);
remote_ct_srcpitch=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.remote_ct_srcpitch = remote_ct_srcpitch; % create struct
temp = reshape([data(find(mod(Count,2)==0),49:50)'],1,[]);
remote_ct_srcroll=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.remote_ct_srcroll = remote_ct_srcroll; % create struct
temp = reshape([data(find(mod(Count,2)==0),51:52)'],1,[]);
remote_ct_srcthrottle=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.remote_ct_srcthrottle = remote_ct_srcthrottle; % create struct
temp = reshape([data(find(mod(Count,2)==0),53:54)'],1,[]);
remote_ct_srcyaw=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.remote_ct_srcyaw = remote_ct_srcyaw; % create struct
temp = reshape([data(find(mod(Count,2)==0),55:56)'],1,[]);
algo_remote_ct_stpitch=double(typecast(uint8(temp),'int16')')/32768*1.0000000000;
TempName.algo_remote_ct_stpitch = algo_remote_ct_stpitch; % create struct
temp = reshape([data(find(mod(Count,2)==0),57:58)'],1,[]);
algo_remote_ct_stroll=double(typecast(uint8(temp),'int16')')/32768*1.0000000000;
TempName.algo_remote_ct_stroll = algo_remote_ct_stroll; % create struct
temp = reshape([data(find(mod(Count,2)==0),59:60)'],1,[]);
algo_remote_ct_stthrottle=double(typecast(uint8(temp),'int16')')/32768*1.0000000000;
TempName.algo_remote_ct_stthrottle = algo_remote_ct_stthrottle; % create struct
temp = reshape([data(find(mod(Count,2)==0),61:62)'],1,[]);
algo_remote_ct_styaw=double(typecast(uint8(temp),'int16')')/32768*1.0000000000;
TempName.algo_remote_ct_styaw = algo_remote_ct_styaw; % create struct
temp = reshape([data(find(mod(Count,4)==3),271:272)'],1,[]);
algo_remote_ct_sttilt_angleIn=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
TempName.algo_remote_ct_sttilt_angleIn = algo_remote_ct_sttilt_angleIn; % create struct
temp = reshape([data(find(mod(Count,2)==0),63:64)'],1,[]);
rtM_ddrowkroll=double(typecast(uint8(temp),'int16')')/32768*4.0000000000;
TempName.rtM_ddrowkroll = rtM_ddrowkroll; % create struct
temp = reshape([data(find(mod(Count,2)==0),65:66)'],1,[]);
rtM_ddrowkpitch=double(typecast(uint8(temp),'int16')')/32768*4.0000000000;
TempName.rtM_ddrowkpitch = rtM_ddrowkpitch; % create struct
temp = reshape([data(find(mod(Count,2)==0),67:68)'],1,[]);
rtM_ddrowkyaw=double(typecast(uint8(temp),'int16')')/32768*4.0000000000;
TempName.rtM_ddrowkyaw = rtM_ddrowkyaw; % create struct
temp = reshape([data(find(mod(Count,2)==0),69:70)'],1,[]);
rtM_ddrowkcurr_vel0=double(typecast(uint8(temp),'int16')')/32768*3000.0000000000;
TempName.rtM_ddrowkcurr_vel0 = rtM_ddrowkcurr_vel0; % create struct
temp = reshape([data(find(mod(Count,2)==0),71:72)'],1,[]);
rtM_ddrowkcurr_vel1=double(typecast(uint8(temp),'int16')')/32768*3000.0000000000;
TempName.rtM_ddrowkcurr_vel1 = rtM_ddrowkcurr_vel1; % create struct
temp = reshape([data(find(mod(Count,2)==0),73:74)'],1,[]);
rtM_ddrowkcurr_vel2=double(typecast(uint8(temp),'int16')')/32768*3000.0000000000;
TempName.rtM_ddrowkcurr_vel2 = rtM_ddrowkcurr_vel2; % create struct
temp = reshape([data(find(mod(Count,2)==0),75:76)'],1,[]);
rtM_ddrowkcurr_pos0=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
TempName.rtM_ddrowkcurr_pos0 = rtM_ddrowkcurr_pos0; % create struct
temp = reshape([data(find(mod(Count,2)==0),77:78)'],1,[]);
rtM_ddrowkcurr_pos1=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
TempName.rtM_ddrowkcurr_pos1 = rtM_ddrowkcurr_pos1; % create struct
temp = reshape([data(find(mod(Count,2)==0),81:82)'],1,[]);
rtM_ddrowkcurr_alt=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
TempName.rtM_ddrowkcurr_alt = rtM_ddrowkcurr_alt; % create struct
temp = reshape([data(find(mod(Count,2)==0),83:84)'],1,[]);
rate_target_ang_vel0=double(typecast(uint8(temp),'int16')')/32768*500.0000000000;
TempName.rate_target_ang_vel0 = rate_target_ang_vel0; % create struct
temp = reshape([data(find(mod(Count,2)==0),85:86)'],1,[]);
rate_target_ang_vel1=double(typecast(uint8(temp),'int16')')/32768*500.0000000000;
TempName.rate_target_ang_vel1 = rate_target_ang_vel1; % create struct
temp = reshape([data(find(mod(Count,2)==0),87:88)'],1,[]);
rate_target_ang_vel2=double(typecast(uint8(temp),'int16')')/32768*500.0000000000;
TempName.rate_target_ang_vel2 = rate_target_ang_vel2; % create struct
temp = reshape([data(find(mod(Count,2)==0),89:90)'],1,[]);
rtM_ddrowkroll_in=double(typecast(uint8(temp),'int16')')/32768*1.1000000000;
TempName.rtM_ddrowkroll_in = rtM_ddrowkroll_in; % create struct
temp = reshape([data(find(mod(Count,2)==0),91:92)'],1,[]);
rtM_ddrowkpitch_in=double(typecast(uint8(temp),'int16')')/32768*1.1000000000;
TempName.rtM_ddrowkpitch_in = rtM_ddrowkpitch_in; % create struct
temp = reshape([data(find(mod(Count,2)==0),93:94)'],1,[]);
rtM_ddrowkyaw_in=double(typecast(uint8(temp),'int16')')/32768*1.1000000000;
TempName.rtM_ddrowkyaw_in = rtM_ddrowkyaw_in; % create struct
temp = reshape([data(find(mod(Count,2)==0),95:96)'],1,[]);
rtM_ddrowkthrottle_in=double(typecast(uint8(temp),'int16')')/32768*1.1000000000;
TempName.rtM_ddrowkthrottle_in = rtM_ddrowkthrottle_in; % create struct
temp = reshape([data(find(mod(Count,2)==0),97:98)'],1,[]);
attitude_target_euler_rate0=double(typecast(uint8(temp),'int16')')/32768*6.3000000000;
TempName.attitude_target_euler_rate0 = attitude_target_euler_rate0; % create struct
temp = reshape([data(find(mod(Count,2)==0),99:100)'],1,[]);
attitude_target_euler_rate1=double(typecast(uint8(temp),'int16')')/32768*6.3000000000;
TempName.attitude_target_euler_rate1 = attitude_target_euler_rate1; % create struct
temp = reshape([data(find(mod(Count,2)==0),101:102)'],1,[]);
attitude_target_euler_rate2=double(typecast(uint8(temp),'int16')')/32768*6.3000000000;
TempName.attitude_target_euler_rate2 = attitude_target_euler_rate2; % create struct
temp = reshape([data(find(mod(Count,2)==0),103:104)'],1,[]);
attitude_target_euler_angle0=double(typecast(uint8(temp),'int16')')/32768*3.1500000000;
TempName.attitude_target_euler_angle0 = attitude_target_euler_angle0; % create struct
temp = reshape([data(find(mod(Count,2)==0),105:106)'],1,[]);
attitude_target_euler_angle1=double(typecast(uint8(temp),'int16')')/32768*3.1500000000;
TempName.attitude_target_euler_angle1 = attitude_target_euler_angle1; % create struct
temp = reshape([data(find(mod(Count,2)==0),107:108)'],1,[]);
attitude_target_euler_angle2=double(typecast(uint8(temp),'int16')')/32768*3.1500000000;
TempName.attitude_target_euler_angle2 = attitude_target_euler_angle2; % create struct
temp = reshape([data(find(mod(Count,2)==1),51:52)'],1,[]);
rtM_ddrowkpwm_out0=double(typecast(uint8(temp),'int16')')/32768*2000.0000000000;
TempName.rtM_ddrowkpwm_out0 = rtM_ddrowkpwm_out0; % create struct
temp = reshape([data(find(mod(Count,2)==1),53:54)'],1,[]);
rtM_ddrowkpwm_out1=double(typecast(uint8(temp),'int16')')/32768*2000.0000000000;
TempName.rtM_ddrowkpwm_out1 = rtM_ddrowkpwm_out1; % create struct
temp = reshape([data(find(mod(Count,2)==1),55:56)'],1,[]);
rtM_ddrowkpwm_out2=double(typecast(uint8(temp),'int16')')/32768*2000.0000000000;
TempName.rtM_ddrowkpwm_out2 = rtM_ddrowkpwm_out2; % create struct
temp = reshape([data(find(mod(Count,2)==1),57:58)'],1,[]);
rtM_ddrowkpwm_out3=double(typecast(uint8(temp),'int16')')/32768*2000.0000000000;
TempName.rtM_ddrowkpwm_out3 = rtM_ddrowkpwm_out3; % create struct
temp = reshape([data(find(mod(Count,2)==1),59:60)'],1,[]);
attitude_error_vector0=double(typecast(uint8(temp),'int16')')/32768*3.1500000000;
TempName.attitude_error_vector0 = attitude_error_vector0; % create struct
temp = reshape([data(find(mod(Count,2)==1),61:62)'],1,[]);
attitude_error_vector1=double(typecast(uint8(temp),'int16')')/32768*3.1500000000;
TempName.attitude_error_vector1 = attitude_error_vector1; % create struct
temp = reshape([data(find(mod(Count,2)==1),63:64)'],1,[]);
attitude_error_vector2=double(typecast(uint8(temp),'int16')')/32768*3.1500000000;
TempName.attitude_error_vector2 = attitude_error_vector2; % create struct
temp = reshape([data(find(mod(Count,2)==1),65:66)'],1,[]);
speed_out0=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.speed_out0 = speed_out0; % create struct
temp = reshape([data(find(mod(Count,2)==1),67:68)'],1,[]);
speed_out1=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.speed_out1 = speed_out1; % create struct
temp = reshape([data(find(mod(Count,2)==1),69:70)'],1,[]);
speed_out2=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.speed_out2 = speed_out2; % create struct
temp = reshape([data(find(mod(Count,2)==1),71:72)'],1,[]);
speed_out3=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.speed_out3 = speed_out3; % create struct
temp = reshape([data(find(mod(Count,2)==1),73:74)'],1,[]);
rtM_ddrowkpos_target0=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
TempName.rtM_ddrowkpos_target0 = rtM_ddrowkpos_target0; % create struct
temp = reshape([data(find(mod(Count,2)==1),75:76)'],1,[]);
rtM_ddrowkpos_target1=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
TempName.rtM_ddrowkpos_target1 = rtM_ddrowkpos_target1; % create struct
temp = reshape([data(find(mod(Count,2)==1),77:78)'],1,[]);
rtM_ddrowkpos_target2=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
TempName.rtM_ddrowkpos_target2 = rtM_ddrowkpos_target2; % create struct
temp = reshape([data(find(mod(Count,2)==1),79:80)'],1,[]);
rtM_ddrowkvel_target0=double(typecast(uint8(temp),'int16')')/32768*1000.0000000000;
TempName.rtM_ddrowkvel_target0 = rtM_ddrowkvel_target0; % create struct
temp = reshape([data(find(mod(Count,2)==1),81:82)'],1,[]);
rtM_ddrowkvel_target1=double(typecast(uint8(temp),'int16')')/32768*1000.0000000000;
TempName.rtM_ddrowkvel_target1 = rtM_ddrowkvel_target1; % create struct
temp = reshape([data(find(mod(Count,2)==1),83:84)'],1,[]);
rtM_ddrowkvel_target2=double(typecast(uint8(temp),'int16')')/32768*1000.0000000000;
TempName.rtM_ddrowkvel_target2 = rtM_ddrowkvel_target2; % create struct
temp = reshape([data(find(mod(Count,2)==1),85:86)'],1,[]);
rtM_ddrowkaccel_target0=double(typecast(uint8(temp),'int16')')/32768*8000.0000000000;
TempName.rtM_ddrowkaccel_target0 = rtM_ddrowkaccel_target0; % create struct
temp = reshape([data(find(mod(Count,2)==1),87:88)'],1,[]);
rtM_ddrowkaccel_target1=double(typecast(uint8(temp),'int16')')/32768*8000.0000000000;
TempName.rtM_ddrowkaccel_target1 = rtM_ddrowkaccel_target1; % create struct
temp = reshape([data(find(mod(Count,2)==1),89:90)'],1,[]);
rtM_ddrowkaccel_target2=double(typecast(uint8(temp),'int16')')/32768*8000.0000000000;
TempName.rtM_ddrowkaccel_target2 = rtM_ddrowkaccel_target2; % create struct
temp = reshape([data(find(mod(Count,2)==1),91:92)'],1,[]);
rtM_ddrowkz_accel_meas=double(typecast(uint8(temp),'int16')')/32768*8000.0000000000;
TempName.rtM_ddrowkz_accel_meas = rtM_ddrowkz_accel_meas; % create struct
temp = reshape([data(find(mod(Count,2)==1),93:94)'],1,[]);
rtM_ddrowkclimb_rate_cms=double(typecast(uint8(temp),'int16')')/32768*500.0000000000;
TempName.rtM_ddrowkclimb_rate_cms = rtM_ddrowkclimb_rate_cms; % create struct
temp = reshape([data(find(mod(Count,2)==1),95:96)'],1,[]);
rtM_ddrowkthrottle_filter=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.rtM_ddrowkthrottle_filter = rtM_ddrowkthrottle_filter; % create struct
temp = reshape([data(find(mod(Count,2)==1),97:98)'],1,[]);
rtM_ddrowkvel_desired_2=double(typecast(uint8(temp),'int16')')/32768*1000.0000000000;
TempName.rtM_ddrowkvel_desired_2 = rtM_ddrowkvel_desired_2; % create struct
temp = reshape([data(find(mod(Count,2)==1),101:102)'],1,[]);
rtM_ddrowknav_pitch_cd=double(typecast(uint8(temp),'int16')')/32768*3600.0000000000;
TempName.rtM_ddrowknav_pitch_cd = rtM_ddrowknav_pitch_cd; % create struct
temp = reshape([data(find(mod(Count,2)==1),103:104)'],1,[]);
rtM_ddrowklatAccDem=double(typecast(uint8(temp),'int16')')/32768*20.0000000000;
TempName.rtM_ddrowklatAccDem = rtM_ddrowklatAccDem; % create struct
temp = reshape([data(find(mod(Count,2)==1),105:106)'],1,[]);
rtM_ddrowkyaw_out=double(typecast(uint8(temp),'int16')')/32768*4500.0000000000;
TempName.rtM_ddrowkyaw_out = rtM_ddrowkyaw_out; % create struct
temp = reshape([data(find(mod(Count,2)==1),107:108)'],1,[]);
rtM_ddrowkthrottle_dem=double(typecast(uint8(temp),'int16')')/32768*1.0000000000;
TempName.rtM_ddrowkthrottle_dem = rtM_ddrowkthrottle_dem; % create struct
temp = reshape([data(find(mod(Count,2)==0),137:138)'],1,[]);
rtM_ddrowkk_rudder=double(typecast(uint8(temp),'int16')')/32768*4500.0000000000;
TempName.rtM_ddrowkk_rudder = rtM_ddrowkk_rudder; % create struct
temp = reshape([data(find(mod(Count,2)==0),139:140)'],1,[]);
rtM_ddrowkk_elevator=double(typecast(uint8(temp),'int16')')/32768*4500.0000000000;
TempName.rtM_ddrowkk_elevator = rtM_ddrowkk_elevator; % create struct
temp = reshape([data(find(mod(Count,2)==0),141:142)'],1,[]);
rtM_ddrowkk_throttle=double(typecast(uint8(temp),'int16')')/32768*1.0000000000;
TempName.rtM_ddrowkk_throttle = rtM_ddrowkk_throttle; % create struct
temp = reshape([data(find(mod(Count,2)==0),143:144)'],1,[]);
rtM_ddrowkk_aileron=double(typecast(uint8(temp),'int16')')/32768*4500.0000000000;
TempName.rtM_ddrowkk_aileron = rtM_ddrowkk_aileron; % create struct
temp = reshape([data(find(mod(Count,2)==1),161:162)'],1,[]);
rtM_ddrowkk_flap=double(typecast(uint8(temp),'int16')')/32768*4500.0000000000;
TempName.rtM_ddrowkk_flap = rtM_ddrowkk_flap; % create struct
temp = reshape([data(find(mod(Count,2)==1),137:138)'],1,[]);
servo_out0=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.servo_out0 = servo_out0; % create struct
temp = reshape([data(find(mod(Count,2)==1),139:140)'],1,[]);
servo_out1=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.servo_out1 = servo_out1; % create struct
temp = reshape([data(find(mod(Count,2)==1),141:142)'],1,[]);
servo_out2=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.servo_out2 = servo_out2; % create struct
temp = reshape([data(find(mod(Count,2)==1),143:144)'],1,[]);
servo_out3=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.servo_out3 = servo_out3; % create struct
temp = reshape([data(find(mod(Count,2)==1),177:178)'],1,[]);
rtM_dworkpitch_dem=double(typecast(uint8(temp),'int16')')/32768*3.1500000000;
TempName.rtM_dworkpitch_dem = rtM_dworkpitch_dem; % create struct
temp = reshape([data(find(mod(Count,2)==1),179:180)'],1,[]);
rtM_dworkhgt_dem=double(typecast(uint8(temp),'int16')')/32768*1000.0000000000;
TempName.rtM_dworkhgt_dem = rtM_dworkhgt_dem; % create struct
temp = reshape([data(find(mod(Count,2)==1),181:182)'],1,[]);
rtM_dworkthrottle_dem=double(typecast(uint8(temp),'int16')')/32768*1.1000000000;
TempName.rtM_dworkthrottle_dem = rtM_dworkthrottle_dem; % create struct
temp = reshape([data(find(mod(Count,2)==1),183:184)'],1,[]);
rtM_dworkaspeed=double(typecast(uint8(temp),'int16')')/32768*50.0000000000;
TempName.rtM_dworkaspeed = rtM_dworkaspeed; % create struct
temp = reshape([data(find(mod(Count,2)==0),195:196)'],1,[]);
rtM_dworkroll_target_pilot=double(typecast(uint8(temp),'int16')')/32768*5000.0000000000;
TempName.rtM_dworkroll_target_pilot = rtM_dworkroll_target_pilot; % create struct
temp = reshape([data(find(mod(Count,2)==1),185:186)'],1,[]);
rtM_dworkpitch_target_pilot=double(typecast(uint8(temp),'int16')')/32768*5000.0000000000;
TempName.rtM_dworkpitch_target_pilot = rtM_dworkpitch_target_pilot; % create struct
temp = reshape([data(find(mod(Count,2)==0),165:168)'],1,[]);
rtM_dworkcurrent_loc0=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.rtM_dworkcurrent_loc0 = rtM_dworkcurrent_loc0; % create struct
temp = reshape([data(find(mod(Count,2)==0),169:172)'],1,[]);
rtM_dworkcurrent_loc1=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.rtM_dworkcurrent_loc1 = rtM_dworkcurrent_loc1; % create struct
temp = reshape([data(find(mod(Count,4)==3),215:216)'],1,[]);
rtM_dworkpos_error0=double(typecast(uint8(temp),'int16')')/32768*1000.0000000000;
TempName.rtM_dworkpos_error0 = rtM_dworkpos_error0; % create struct
temp = reshape([data(find(mod(Count,4)==3),217:218)'],1,[]);
rtM_dworkpos_error1=double(typecast(uint8(temp),'int16')')/32768*1000.0000000000;
TempName.rtM_dworkpos_error1 = rtM_dworkpos_error1; % create struct
temp = reshape([data(find(mod(Count,4)==3),219:220)'],1,[]);
rtM_dworkpos_error2=double(typecast(uint8(temp),'int16')')/32768*1000.0000000000;
TempName.rtM_dworkpos_error2 = rtM_dworkpos_error2; % create struct
temp = reshape([data(find(mod(Count,4)==1),221:222)'],1,[]);
PathModeOut_slheightCmd=double(typecast(uint8(temp),'int16')')/32768*30000.0000000000;
TempName.PathModeOut_slheightCmd = PathModeOut_slheightCmd; % create struct
temp = reshape([data(find(mod(Count,4)==2),221:222)'],1,[]);
PathModeOut_slmaxClimbSpeed=double(typecast(uint8(temp),'int16')')/32768*500.0000000000;
TempName.PathModeOut_slmaxClimbSpeed = PathModeOut_slmaxClimbSpeed; % create struct
temp = reshape([data(find(mod(Count,4)==3),209:210)'],1,[]);
PathModeOut_slflightTaskMode=double(typecast(uint8(temp),'int16')')/1*1.0000000000;
TempName.PathModeOut_slflightTaskMode = PathModeOut_slflightTaskMode; % create struct
temp = reshape([data(find(mod(Count,16)==10),291:291)'],1,[]);
rtM_ddworkplane_mode=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.rtM_ddworkplane_mode = rtM_ddworkplane_mode; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */							   
temp = reshape([data(find(mod(Count,1)==0),243:244)'],1,[]);
speed_out_pre0=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.speed_out_pre0 = speed_out_pre0; % create struct
temp = reshape([data(find(mod(Count,1)==0),245:246)'],1,[]);
speed_out_pre1=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.speed_out_pre1 = speed_out_pre1; % create struct
temp = reshape([data(find(mod(Count,1)==0),247:248)'],1,[]);
speed_out_pre2=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.speed_out_pre2 = speed_out_pre2; % create struct
temp = reshape([data(find(mod(Count,1)==0),249:250)'],1,[]);
speed_out_pre3=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.speed_out_pre3 = speed_out_pre3; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,8)==2),197:198)'],1,[]);
mission_itemseq=double(typecast(uint8(temp),'int16')')/1*1.0000000000;
TempName.mission_itemseq = mission_itemseq; % create struct
temp = reshape([data(find(mod(Count,8)==2),199:202)'],1,[]);
mission_itemlat=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
TempName.mission_itemlat = mission_itemlat; % create struct
temp = reshape([data(find(mod(Count,8)==2),203:206)'],1,[]);
mission_itemlon=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
TempName.mission_itemlon = mission_itemlon; % create struct
temp = reshape([data(find(mod(Count,8)==3),197:198)'],1,[]);
rtM_dworklocnum=double(typecast(uint8(temp),'int16')')/32768*1.0000000000;
TempName.rtM_dworklocnum = rtM_dworklocnum; % create struct
temp = reshape([data(find(mod(Count,8)==3),199:202)'],1,[]);
rtM_dworkloclat=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
TempName.rtM_dworkloclat = rtM_dworkloclat; % create struct
temp = reshape([data(find(mod(Count,8)==3),203:206)'],1,[]);
rtM_dworkloclon=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
TempName.rtM_dworkloclon = rtM_dworkloclon; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,4)==0),209:212)'],1,[]);
algo_NAV_lond=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
TempName.algo_NAV_lond = algo_NAV_lond; % create struct
temp = reshape([data(find(mod(Count,4)==0),213:216)'],1,[]);
algo_NAV_latd=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
TempName.algo_NAV_latd = algo_NAV_latd; % create struct
temp = reshape([data(find(mod(Count,4)==1),209:210)'],1,[]);
algo_NAV_yawd=double(typecast(uint8(temp),'int16')')/32768*200.0000000000;
TempName.algo_NAV_yawd = algo_NAV_yawd; % create struct
temp = reshape([data(find(mod(Count,4)==1),211:212)'],1,[]);
algo_NAV_pitchd=double(typecast(uint8(temp),'int16')')/32768*200.0000000000;
TempName.algo_NAV_pitchd = algo_NAV_pitchd; % create struct
temp = reshape([data(find(mod(Count,4)==1),213:214)'],1,[]);
algo_NAV_rolld=double(typecast(uint8(temp),'int16')')/32768*200.0000000000;
TempName.algo_NAV_rolld = algo_NAV_rolld; % create struct
temp = reshape([data(find(mod(Count,4)==1),219:220)'],1,[]);
algo_NAV_alt=double(typecast(uint8(temp),'int16')')/32768*2000.0000000000;
TempName.algo_NAV_alt = algo_NAV_alt; % create struct
temp = reshape([data(find(mod(Count,4)==2),209:210)'],1,[]);
algo_NAV_Vn=double(typecast(uint8(temp),'int16')')/32768*100.0000000000;
TempName.algo_NAV_Vn = algo_NAV_Vn; % create struct
temp = reshape([data(find(mod(Count,4)==2),211:212)'],1,[]);
algo_NAV_Ve=double(typecast(uint8(temp),'int16')')/32768*100.0000000000;
TempName.algo_NAV_Ve = algo_NAV_Ve; % create struct
temp = reshape([data(find(mod(Count,4)==2),213:214)'],1,[]);
algo_NAV_Vd=double(typecast(uint8(temp),'int16')')/32768*100.0000000000;
TempName.algo_NAV_Vd = algo_NAV_Vd; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,4)==0),209:210)'],1,[]);
rtU_MagYawacc0=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.rtU_MagYawacc0 = rtU_MagYawacc0; % create struct
temp = reshape([data(find(mod(Count,4)==0),211:212)'],1,[]);
rtU_MagYawacc1=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.rtU_MagYawacc1 = rtU_MagYawacc1; % create struct
temp = reshape([data(find(mod(Count,4)==0),213:214)'],1,[]);
rtU_MagYawacc2=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.rtU_MagYawacc2 = rtU_MagYawacc2; % create struct
temp = reshape([data(find(mod(Count,4)==0),215:216)'],1,[]);
rtU_MagYawmag0=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.rtU_MagYawmag0 = rtU_MagYawmag0; % create struct
temp = reshape([data(find(mod(Count,4)==0),217:218)'],1,[]);
rtU_MagYawmag1=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.rtU_MagYawmag1 = rtU_MagYawmag1; % create struct
temp = reshape([data(find(mod(Count,4)==0),219:220)'],1,[]);
rtU_MagYawmag2=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.rtU_MagYawmag2 = rtU_MagYawmag2; % create struct
temp = reshape([data(find(mod(Count,4)==0),221:222)'],1,[]);
rtU_MagYawout=double(typecast(uint8(temp),'int16')')/32768*3.1500000000;
TempName.rtU_MagYawout = rtU_MagYawout; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,2)==0),159:160)'],1,[]);
rtY_filter3gx=double(typecast(uint8(temp),'int16')')/32768*17.5000000000;
TempName.rtY_filter3gx = rtY_filter3gx; % create struct
temp = reshape([data(find(mod(Count,2)==0),161:162)'],1,[]);
rtY_filter4gy=double(typecast(uint8(temp),'int16')')/32768*17.5000000000;
TempName.rtY_filter4gy = rtY_filter4gy; % create struct
temp = reshape([data(find(mod(Count,2)==0),163:164)'],1,[]);
rtY_filter5gz=double(typecast(uint8(temp),'int16')')/32768*17.5000000000;
TempName.rtY_filter5gz = rtY_filter5gz; % create struct
temp = reshape([data(find(mod(Count,2)==1),145:146)'],1,[]);
rtY_filter0ax=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.rtY_filter0ax = rtY_filter0ax; % create struct
temp = reshape([data(find(mod(Count,2)==1),147:148)'],1,[]);
rtY_filter1ay=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.rtY_filter1ay = rtY_filter1ay; % create struct
temp = reshape([data(find(mod(Count,2)==1),149:150)'],1,[]);
rtY_filter2az=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.rtY_filter2az = rtY_filter2az; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,16)==0),289:289)'],1,[]);
ASOC=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.ASOC = ASOC; % create struct
temp = reshape([data(find(mod(Count,16)==0),290:290)'],1,[]);
RSOC=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.RSOC = RSOC; % create struct
temp = reshape([data(find(mod(Count,16)==0),291:292)'],1,[]);
SOH=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.SOH = SOH; % create struct
temp = reshape([data(find(mod(Count,16)==1),289:290)'],1,[]);
Average_Time_To_empty=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Average_Time_To_empty = Average_Time_To_empty; % create struct
temp = reshape([data(find(mod(Count,16)==1),291:292)'],1,[]);
Cell_Volt0=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cell_Volt0 = Cell_Volt0; % create struct
temp = reshape([data(find(mod(Count,16)==2),289:290)'],1,[]);
Cell_Volt1=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cell_Volt1 = Cell_Volt1; % create struct
temp = reshape([data(find(mod(Count,16)==2),291:292)'],1,[]);
Cell_Volt2=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cell_Volt2 = Cell_Volt2; % create struct
temp = reshape([data(find(mod(Count,16)==3),289:290)'],1,[]);
Cell_Volt3=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cell_Volt3 = Cell_Volt3; % create struct
temp = reshape([data(find(mod(Count,16)==3),291:292)'],1,[]);
Cell_Volt4=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cell_Volt4 = Cell_Volt4; % create struct
temp = reshape([data(find(mod(Count,16)==4),289:290)'],1,[]);
Cell_Volt5=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cell_Volt5 = Cell_Volt5; % create struct
temp = reshape([data(find(mod(Count,16)==4),291:292)'],1,[]);
Cycle_Count=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cycle_Count = Cycle_Count; % create struct
temp = reshape([data(find(mod(Count,16)==5),289:290)'],1,[]);
Design_Cap=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Design_Cap = Design_Cap; % create struct
temp = reshape([data(find(mod(Count,16)==5),291:292)'],1,[]);
Design_Volt=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Design_Volt = Design_Volt; % create struct
temp = reshape([data(find(mod(Count,16)==6),289:290)'],1,[]);
FCC=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.FCC = FCC; % create struct
temp = reshape([data(find(mod(Count,16)==6),291:292)'],1,[]);
FW_Version=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.FW_Version = FW_Version; % create struct
temp = reshape([data(find(mod(Count,16)==7),289:290)'],1,[]);
Manufacture_Date=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Manufacture_Date = Manufacture_Date; % create struct
temp = reshape([data(find(mod(Count,16)==7),291:292)'],1,[]);
Other_Status=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Other_Status = Other_Status; % create struct
temp = reshape([data(find(mod(Count,16)==8),289:290)'],1,[]);
Rem_Cap=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Rem_Cap = Rem_Cap; % create struct
temp = reshape([data(find(mod(Count,16)==8),291:292)'],1,[]);
Run_Time_TO_Empty=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Run_Time_TO_Empty = Run_Time_TO_Empty; % create struct
temp = reshape([data(find(mod(Count,16)==9),289:290)'],1,[]);
Safty_Status=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Safty_Status = Safty_Status; % create struct
temp = reshape([data(find(mod(Count,16)==9),291:292)'],1,[]);
Serial_Num=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Serial_Num = Serial_Num; % create struct
temp = reshape([data(find(mod(Count,2)==0),145:146)'],1,[]);
Temperature=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Temperature = Temperature; % create struct
temp = reshape([data(find(mod(Count,2)==0),147:148)'],1,[]);
Total_Cycles=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Total_Cycles = Total_Cycles; % create struct
temp = reshape([data(find(mod(Count,2)==0),149:150)'],1,[]);
Voltage=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Voltage = Voltage; % create struct
temp = reshape([data(find(mod(Count,2)==0),151:154)'],1,[]);
Now_Current=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.Now_Current = Now_Current; % create struct
temp = reshape([data(find(mod(Count,2)==0),155:158)'],1,[]);
Average_Current=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.Average_Current = Average_Current; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,2)==0),177:178)'],1,[]);
diff_press_filtered_pa=double(typecast(uint8(temp),'int16')')/32768*1000.0000000000;
TempName.diff_press_filtered_pa = diff_press_filtered_pa; % create struct
temp = reshape([data(find(mod(Count,2)==0),179:180)'],1,[]);
indicated_airspeed=double(typecast(uint8(temp),'int16')')/32768*100.0000000000;
TempName.indicated_airspeed = indicated_airspeed; % create struct
temp = reshape([data(find(mod(Count,2)==0),181:182)'],1,[]);
true_airspeed=double(typecast(uint8(temp),'int16')')/32768*100.0000000000;
TempName.true_airspeed = true_airspeed; % create struct
temp = reshape([data(find(mod(Count,2)==0),183:184)'],1,[]);
EAS_Algo=double(typecast(uint8(temp),'int16')')/32768*50.0000000000;
TempName.EAS_Algo = EAS_Algo; % create struct
temp = reshape([data(find(mod(Count,2)==0),185:186)'],1,[]);
EAS2TAS_Algo=double(typecast(uint8(temp),'int16')')/32768*2.0000000000;
TempName.EAS2TAS_Algo = EAS2TAS_Algo; % create struct
temp = reshape([data(find(mod(Count,2)==0),187:188)'],1,[]);
airspeed_temperature=double(typecast(uint8(temp),'int16')')/32768*200.0000000000;
TempName.airspeed_temperature = airspeed_temperature; % create struct
temp = reshape([data(find(mod(Count,2)==0),189:190)'],1,[]);
airspeed_used=double(typecast(uint8(temp),'int16')')/32768*100.0000000000;
TempName.airspeed_used = airspeed_used; % create struct
temp = reshape([data(find(mod(Count,2)==0),191:192)'],1,[]);
diff_press_pa_raw=double(typecast(uint8(temp),'int16')')/32768*1000.0000000000;
TempName.diff_press_pa_raw = diff_press_pa_raw; % create struct
temp = reshape([data(find(mod(Count,2)==0),193:194)'],1,[]);
diff_offset=double(typecast(uint8(temp),'int16')')/32768*300.0000000000;
TempName.diff_offset = diff_offset; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,2)==1),155:155)'],1,[]);
mag_retry_count0=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.mag_retry_count0 = mag_retry_count0; % create struct
temp = reshape([data(find(mod(Count,2)==1),156:156)'],1,[]);
mag_retry_count1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.mag_retry_count1 = mag_retry_count1; % create struct
temp = reshape([data(find(mod(Count,2)==1),157:157)'],1,[]);
mag_retry_count2=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.mag_retry_count2 = mag_retry_count2; % create struct
temp = reshape([data(find(mod(Count,2)==1),158:158)'],1,[]);
cnt_fs0=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.cnt_fs0 = cnt_fs0; % create struct
temp = reshape([data(find(mod(Count,2)==1),159:159)'],1,[]);
cnt_fs1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.cnt_fs1 = cnt_fs1; % create struct
temp = reshape([data(find(mod(Count,2)==1),160:160)'],1,[]);
cnt_fs2=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.cnt_fs2 = cnt_fs2; % create struct
temp = reshape([data(find(mod(Count,4)==3),287:288)'],1,[]);
mag_systime=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.mag_systime = mag_systime; % create struct
temp = reshape([data(find(mod(Count,2)==1),163:163)'],1,[]);
air_retry_count=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.air_retry_count = air_retry_count; % create struct
temp = reshape([data(find(mod(Count,2)==1),173:173)'],1,[]);
mag_reg_err_cnt_0=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.mag_reg_err_cnt_0 = mag_reg_err_cnt_0; % create struct
temp = reshape([data(find(mod(Count,2)==1),174:174)'],1,[]);
mag_reg_err_cnt_1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.mag_reg_err_cnt_1 = mag_reg_err_cnt_1; % create struct
temp = reshape([data(find(mod(Count,2)==1),175:175)'],1,[]);
mag_reg_err_cnt_2=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.mag_reg_err_cnt_2 = mag_reg_err_cnt_2; % create struct
temp = reshape([data(find(mod(Count,2)==1),176:176)'],1,[]);
mag_data_err_cnt_0=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.mag_data_err_cnt_0 = mag_data_err_cnt_0; % create struct
temp = reshape([data(find(mod(Count,2)==1),173:173)'],1,[]);
mag_data_err_cnt_1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.mag_data_err_cnt_1 = mag_data_err_cnt_1; % create struct
temp = reshape([data(find(mod(Count,2)==1),174:174)'],1,[]);
mag_data_err_cnt_2=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.mag_data_err_cnt_2 = mag_data_err_cnt_2; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,8)==0),197:198)'],1,[]);
IMUTmp=double(typecast(uint8(temp),'int16')')/32768*1.0000000000;
TempName.IMUTmp = IMUTmp; % create struct
temp = reshape([data(find(mod(Count,8)==0),199:200)'],1,[]);
pwm_io_cnt=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.pwm_io_cnt = pwm_io_cnt; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,8)==4),197:198)'],1,[]);
x=double(typecast(uint8(temp),'int16')')/32768*180.0000000000;
TempName.x = x; % create struct
temp = reshape([data(find(mod(Count,8)==4),199:200)'],1,[]);
y=double(typecast(uint8(temp),'int16')')/32768*180.0000000000;
TempName.y = y; % create struct
temp = reshape([data(find(mod(Count,8)==4),201:202)'],1,[]);
z=double(typecast(uint8(temp),'int16')')/32768*5000.0000000000;
TempName.z = z; % create struct
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,16)==4),243:246)'],1,[]);
fc_sn_num=double(typecast(uint8(temp),'uint32')')/10000000*1.0000000000;
TempName.fc_sn_num = fc_sn_num; % create struct
% /* +===========+================================+=============+============+==============+ */
% /* ------------------------------algo sl imu1 log -----------------------------------------  */20200401
temp = reshape([data(find(mod(Count,1)==0),27:28)'],1,[]);
algo_imu_filtax=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.algo_imu_filtax = algo_imu_filtax; % create struct
temp = reshape([data(find(mod(Count,1)==0),29:30)'],1,[]);
algo_imu_filtay=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.algo_imu_filtay = algo_imu_filtay; % create struct
temp = reshape([data(find(mod(Count,1)==0),31:32)'],1,[]);
algo_imu_filtaz=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.algo_imu_filtaz = algo_imu_filtaz; % create struct
temp = reshape([data(find(mod(Count,1)==0),33:34)'],1,[]);
algo_imu_filtgx=double(typecast(uint8(temp),'int16')')/32768*17.5000000000;
TempName.algo_imu_filtgx = algo_imu_filtgx; % create struct
temp = reshape([data(find(mod(Count,1)==0),35:36)'],1,[]);
algo_imu_filtgy=double(typecast(uint8(temp),'int16')')/32768*17.5000000000;
TempName.algo_imu_filtgy = algo_imu_filtgy; % create struct
temp = reshape([data(find(mod(Count,1)==0),37:38)'],1,[]);
algo_imu_filtgz=double(typecast(uint8(temp),'int16')')/32768*17.5000000000;
TempName.algo_imu_filtgz = algo_imu_filtgz; % create struct
% /* ------------------------------algo sl imu2 log -----------------------------------------  */20200623
temp = reshape([data(find(mod(Count,4)==1),277:278)'],1,[]);
algo_imu2_filtax=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.algo_imu2_filtax = algo_imu2_filtax; % create struct
temp = reshape([data(find(mod(Count,4)==1),279:280)'],1,[]);
algo_imu2_filtay=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.algo_imu2_filtay = algo_imu2_filtay; % create struct
temp = reshape([data(find(mod(Count,4)==1),281:282)'],1,[]);
algo_imu2_filtaz=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.algo_imu2_filtaz = algo_imu2_filtaz; % create struct
temp = reshape([data(find(mod(Count,4)==1),283:284)'],1,[]);
algo_imu2_filtgx=double(typecast(uint8(temp),'int16')')/32768*17.5000000000;
TempName.algo_imu2_filtgx = algo_imu2_filtgx; % create struct
temp = reshape([data(find(mod(Count,4)==1),285:286)'],1,[]);
algo_imu2_filtgy=double(typecast(uint8(temp),'int16')')/32768*17.5000000000;
TempName.algo_imu2_filtgy = algo_imu2_filtgy; % create struct
temp = reshape([data(find(mod(Count,4)==1),287:288)'],1,[]);
algo_imu2_filtgz=double(typecast(uint8(temp),'int16')')/32768*17.5000000000;
TempName.algo_imu2_filtgz = algo_imu2_filtgz; % create struct
% /* ------------------------------algo sl imu3 log -----------------------------------------  */20200623
temp = reshape([data(find(mod(Count,4)==2),277:278)'],1,[]);
algo_imu3_filtax=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.algo_imu3_filtax = algo_imu3_filtax; % create struct
temp = reshape([data(find(mod(Count,4)==2),279:280)'],1,[]);
algo_imu3_filtay=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.algo_imu3_filtay = algo_imu3_filtay; % create struct
temp = reshape([data(find(mod(Count,4)==2),281:282)'],1,[]);
algo_imu3_filtaz=double(typecast(uint8(temp),'int16')')/32768*80.0000000000;
TempName.algo_imu3_filtaz = algo_imu3_filtaz; % create struct
temp = reshape([data(find(mod(Count,4)==2),283:284)'],1,[]);
algo_imu3_filtgx=double(typecast(uint8(temp),'int16')')/32768*17.5000000000;
TempName.algo_imu3_filtgx = algo_imu3_filtgx; % create struct
temp = reshape([data(find(mod(Count,4)==2),285:286)'],1,[]);
algo_imu3_filtgy=double(typecast(uint8(temp),'int16')')/32768*17.5000000000;
TempName.algo_imu3_filtgy = algo_imu3_filtgy; % create struct
temp = reshape([data(find(mod(Count,4)==2),287:288)'],1,[]);
algo_imu3_filtgz=double(typecast(uint8(temp),'int16')')/32768*17.5000000000;
TempName.algo_imu3_filtgz = algo_imu3_filtgz; % create struct
% /* ------------------------------algo sl  log --------------------------------------------- */20200312
% /*-----------------------RefModel_SystemArchitecture_U.IN_MAVLINK.------------------------- */
% /* |@@SL.mavlink_msg_groundHomeLLA@@+-----------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,8)==4),203:204)'],1,[]);
mavlink_msg_groundHomeLLA0=double(typecast(uint8(temp),'int16')')/32768*180.0000000000;
SL.mavlink_msg_groundHomeLLA.mavlink_msg_groundHomeLLA0 = mavlink_msg_groundHomeLLA0; % create struct
temp = reshape([data(find(mod(Count,8)==4),205:206)'],1,[]);
mavlink_msg_groundHomeLLA1=double(typecast(uint8(temp),'int16')')/32768*180.0000000000;
SL.mavlink_msg_groundHomeLLA.mavlink_msg_groundHomeLLA1 = mavlink_msg_groundHomeLLA1; % create struct
temp = reshape([data(find(mod(Count,8)==4),207:208)'],1,[]);
mavlink_msg_groundHomeLLA2=double(typecast(uint8(temp),'int16')')/32768*5000.0000000000;
SL.mavlink_msg_groundHomeLLA.mavlink_msg_groundHomeLLA2 = mavlink_msg_groundHomeLLA2; % create struct
% /*-----------------------RefModel_SystemArchitecture_U.IN_MAVLINK.------------------------- */
% /* |@@SL.mavlink_msg_command_battery_data@@+----+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,8)==4),293:294)'],1,[]);
fullCapacity=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.mavlink_msg_command_battery_data.fullCapacity = fullCapacity; % create struct
temp = reshape([data(find(mod(Count,8)==5),293:293)'],1,[]);
lifePercent=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.mavlink_msg_command_battery_data.lifePercent = lifePercent; % create struct
temp = reshape([data(find(mod(Count,8)==6),293:294)'],1,[]);
cycleTime=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.mavlink_msg_command_battery_data.cycleTime = cycleTime; % create struct
temp = reshape([data(find(mod(Count,8)==7),293:294)'],1,[]);
batteryId=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.mavlink_msg_command_battery_data.batteryId = batteryId; % create struct
% /* |@@SL.mavlink_mission_item_def@@+------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,8)==3),197:198)'],1,[]);
seq=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.mavlink_mission_item_def.seq = seq; % create struct
temp = reshape([data(find(mod(Count,8)==3),199:202)'],1,[]);
x=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.mavlink_mission_item_def.x = x; % create struct
temp = reshape([data(find(mod(Count,8)==3),203:206)'],1,[]);
y=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.mavlink_mission_item_def.y = y; % create struct
temp = reshape([data(find(mod(Count,8)==3),207:208)'],1,[]);
z=double(typecast(uint8(temp),'int16')')/32768*5000.0000000000;
SL.mavlink_mission_item_def.z = z; % create struct
% /*-----------------------RefModel_SystemArchitecture_Y.OUT_SensorSignalIntegrity.-----------*/
% /* |@@SL.SensorSelect@@+------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,16)==0),223:224)'],1,[]);
IMU=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorSelect.IMU = IMU; % create struct
temp = reshape([data(find(mod(Count,16)==0),225:226)'],1,[]);
Mag=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorSelect.Mag = Mag; % create struct
temp = reshape([data(find(mod(Count,16)==0),227:228)'],1,[]);
GPS=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorSelect.GPS = GPS; % create struct
temp = reshape([data(find(mod(Count,16)==1),223:224)'],1,[]);
Baro=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorSelect.Baro = Baro; % create struct
temp = reshape([data(find(mod(Count,16)==1),225:226)'],1,[]);
Radar=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorSelect.Radar = Radar; % create struct
temp = reshape([data(find(mod(Count,16)==1),227:228)'],1,[]);
Camera=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorSelect.Camera = Camera; % create struct
temp = reshape([data(find(mod(Count,16)==2),223:224)'],1,[]);
Lidar=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorSelect.Lidar = Lidar; % create struct
% /* |@@SL.SensorUpdateFlag@@+--------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,16)==2),225:225)'],1,[]);
mag1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorUpdateFlag.mag1 = mag1; % create struct
temp = reshape([data(find(mod(Count,16)==2),226:226)'],1,[]);
mag2=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorUpdateFlag.mag2 = mag2; % create struct
temp = reshape([data(find(mod(Count,16)==2),227:227)'],1,[]);
um482=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorUpdateFlag.um482 = um482; % create struct
temp = reshape([data(find(mod(Count,16)==2),228:228)'],1,[]);
airspeed1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorUpdateFlag.airspeed1 = airspeed1; % create struct
temp = reshape([data(find(mod(Count,16)==3),223:223)'],1,[]);
ublox1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorUpdateFlag.ublox1 = ublox1; % create struct
temp = reshape([data(find(mod(Count,16)==3),224:224)'],1,[]);
radar1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorUpdateFlag.radar1 = radar1; % create struct
temp = reshape([data(find(mod(Count,16)==3),225:225)'],1,[]);
IMU1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorUpdateFlag.IMU1 = IMU1; % create struct
temp = reshape([data(find(mod(Count,16)==3),226:226)'],1,[]);
IMU2=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorUpdateFlag.IMU2 = IMU2; % create struct
temp = reshape([data(find(mod(Count,16)==3),227:227)'],1,[]);
IMU3=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorUpdateFlag.IMU3 = IMU3; % create struct
temp = reshape([data(find(mod(Count,16)==3),228:228)'],1,[]);
baro1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorUpdateFlag.baro1 = baro1; % create struct
% /* |@@SL.SensorLosttime@@+----------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,16)==4),223:224)'],1,[]);
mag1=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorLosttime.mag1 = mag1; % create struct
temp = reshape([data(find(mod(Count,16)==4),225:226)'],1,[]);
mag2=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorLosttime.mag2 = mag2; % create struct
temp = reshape([data(find(mod(Count,16)==4),227:228)'],1,[]);
um482=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorLosttime.um482 = um482; % create struct
temp = reshape([data(find(mod(Count,16)==5),223:224)'],1,[]);
airspeed1=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorLosttime.airspeed1 = airspeed1; % create struct
temp = reshape([data(find(mod(Count,16)==5),225:226)'],1,[]);
radar1=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorLosttime.radar1 = radar1; % create struct
temp = reshape([data(find(mod(Count,16)==5),227:228)'],1,[]);
ublox1=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorLosttime.ublox1 = ublox1; % create struct
temp = reshape([data(find(mod(Count,16)==6),223:224)'],1,[]);
IMU1=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorLosttime.IMU1 = IMU1; % create struct
temp = reshape([data(find(mod(Count,16)==6),225:226)'],1,[]);
IMU2=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorLosttime.IMU2 = IMU2; % create struct
temp = reshape([data(find(mod(Count,16)==6),227:228)'],1,[]);
IMU3=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorLosttime.IMU3 = IMU3; % create struct
temp = reshape([data(find(mod(Count,16)==7),223:224)'],1,[]);
baro1=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SensorLosttime.baro1 = baro1; % create struct
% /* |@@SL.SensorStatus@@+------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,16)==7),225:225)'],1,[]);
mag1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorStatus.mag1 = mag1; % create struct
temp = reshape([data(find(mod(Count,16)==7),226:226)'],1,[]);
mag2=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorStatus.mag2 = mag2; % create struct
temp = reshape([data(find(mod(Count,16)==7),227:227)'],1,[]);
um482=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorStatus.um482 = um482; % create struct
temp = reshape([data(find(mod(Count,16)==7),228:228)'],1,[]);
airspeed1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorStatus.airspeed1 = airspeed1; % create struct
temp = reshape([data(find(mod(Count,16)==8),223:223)'],1,[]);
radar1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorStatus.radar1 = radar1; % create struct
temp = reshape([data(find(mod(Count,16)==8),224:224)'],1,[]);
ublox1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorStatus.ublox1 = ublox1; % create struct
temp = reshape([data(find(mod(Count,16)==8),225:225)'],1,[]);
IMU1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorStatus.IMU1 = IMU1; % create struct
temp = reshape([data(find(mod(Count,16)==8),226:226)'],1,[]);
IMU2=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorStatus.IMU2 = IMU2; % create struct
temp = reshape([data(find(mod(Count,16)==8),227:227)'],1,[]);
IMU3=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorStatus.IMU3 = IMU3; % create struct
temp = reshape([data(find(mod(Count,16)==8),228:228)'],1,[]);
baro1=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorStatus.baro1 = baro1; % create struct
temp = reshape([data(find(mod(Count,16)==9),223:223)'],1,[]);
SystemHealthStatus=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.SensorStatus.SystemHealthStatus = SystemHealthStatus; % create struct
% /*             ----------RefModel_SystemArchitecture_Y.OUT_PAYLOAD.-----------------------| */										
% /* |@@SL.CAMERA@@-------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,16)==9),225:226)'],1,[]);
time=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.CAMERA.time = time; % create struct
temp = reshape([data(find(mod(Count,16)==9),227:228)'],1,[]);
trigger=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.CAMERA.trigger = trigger; % create struct
temp = reshape([data(find(mod(Count,16)==10),223:226)'],1,[]);
LLA0=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.CAMERA.LLA0 = LLA0; % create struct
temp = reshape([data(find(mod(Count,16)==15),223:226)'],1,[]);
LLA1=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.CAMERA.LLA1 = LLA1; % create struct
temp = reshape([data(find(mod(Count,16)==15),227:228)'],1,[]);
LLA2=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.CAMERA.LLA2 = LLA2; % create struct
temp = reshape([data(find(mod(Count,16)==11),223:224)'],1,[]);
groundspeed=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.CAMERA.groundspeed = groundspeed; % create struct
% /* |@@SL.LIDAR@@+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,16)==11),225:226)'],1,[]);
time=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.LIDAR.time = time; % create struct
temp = reshape([data(find(mod(Count,16)==11),227:228)'],1,[]);
trigger=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.LIDAR.trigger = trigger; % create struct
temp = reshape([data(find(mod(Count,16)==12),223:224)'],1,[]);
LLA0=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.LIDAR.LLA0 = LLA0; % create struct
temp = reshape([data(find(mod(Count,16)==12),225:226)'],1,[]);
LLA1=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.LIDAR.LLA1 = LLA1; % create struct
temp = reshape([data(find(mod(Count,16)==12),227:228)'],1,[]);
LLA2=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.LIDAR.LLA2 = LLA2; % create struct
temp = reshape([data(find(mod(Count,16)==13),223:224)'],1,[]);
groundspeed=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.LIDAR.groundspeed = groundspeed; % create struct
% /*             ----------RefModel_SystemArchitecture_Y.OUT_BATTERY.-----------------------| */										
% /* |@@SL.PowerConsume@@-------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,16)==5),243:244)'],1,[]);
AllTheTimeVoltage=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.PowerConsume.AllTheTimeVoltage = AllTheTimeVoltage; % create struct
temp = reshape([data(find(mod(Count,16)==6),243:246)'],1,[]);
AllTheTimeCurrent=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
SL.PowerConsume.AllTheTimeCurrent = AllTheTimeCurrent; % create struct
temp = reshape([data(find(mod(Count,16)==5),245:245)'],1,[]);
AllTheTimePowerConsume=double(typecast(uint8(temp),'int8')')/1*1.0000000000;
SL.PowerConsume.AllTheTimePowerConsume = AllTheTimePowerConsume; % create struct
temp = reshape([data(find(mod(Count,16)==5),246:246)'],1,[]);
GroundStandby=double(typecast(uint8(temp),'int8')')/1*1.0000000000;
SL.PowerConsume.GroundStandby = GroundStandby; % create struct
temp = reshape([data(find(mod(Count,16)==7),243:243)'],1,[]);
TakeOff=double(typecast(uint8(temp),'int8')')/1*1.0000000000;
SL.PowerConsume.TakeOff = TakeOff; % create struct
temp = reshape([data(find(mod(Count,16)==7),244:244)'],1,[]);
HoverAdjust=double(typecast(uint8(temp),'int8')')/1*1.0000000000;
SL.PowerConsume.HoverAdjust = HoverAdjust; % create struct
temp = reshape([data(find(mod(Count,16)==7),245:245)'],1,[]);
Rotor2fix=double(typecast(uint8(temp),'int8')')/1*1.0000000000;
SL.PowerConsume.Rotor2fix = Rotor2fix; % create struct
temp = reshape([data(find(mod(Count,16)==7),246:246)'],1,[]);
HoverUp=double(typecast(uint8(temp),'int8')')/1*1.0000000000;
SL.PowerConsume.HoverUp = HoverUp; % create struct
temp = reshape([data(find(mod(Count,16)==8),243:243)'],1,[]);
PathFollow=double(typecast(uint8(temp),'int8')')/1*1.0000000000;
SL.PowerConsume.PathFollow = PathFollow; % create struct
temp = reshape([data(find(mod(Count,16)==8),244:244)'],1,[]);
GoHome=double(typecast(uint8(temp),'int8')')/1*1.0000000000;
SL.PowerConsume.GoHome = GoHome; % create struct
temp = reshape([data(find(mod(Count,16)==8),245:245)'],1,[]);
HoverDown=double(typecast(uint8(temp),'int8')')/1*1.0000000000;
SL.PowerConsume.HoverDown = HoverDown; % create struct
temp = reshape([data(find(mod(Count,16)==8),246:246)'],1,[]);
Fix2Rotor=double(typecast(uint8(temp),'int8')')/1*1.0000000000;
SL.PowerConsume.Fix2Rotor = Fix2Rotor; % create struct
temp = reshape([data(find(mod(Count,16)==9),243:243)'],1,[]);
Land=double(typecast(uint8(temp),'int8')')/1*1.0000000000;
SL.PowerConsume.Land = Land; % create struct
% /*-----------------------SimParam_LLA    -------------------------------------------------| */										
% /* |@@SL.SimParam_LLA@@-------------------------+-------------+-------------+-------------| */
temp = reshape([data(find(mod(Count,16)==13),225:226)'],1,[]);
SimParam_LLA0=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SimParam_LLA.SimParam_LLA0 = SimParam_LLA0; % create struct
temp = reshape([data(find(mod(Count,16)==13),227:228)'],1,[]);
SimParam_LLA1=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SimParam_LLA.SimParam_LLA1 = SimParam_LLA1; % create struct
temp = reshape([data(find(mod(Count,16)==14),223:224)'],1,[]);
SimParam_LLA2=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.SimParam_LLA.SimParam_LLA2 = SimParam_LLA2; % create struct
% /*-----------------------Debug_Task_RTInfo    ----------------------------------------------*/										
% /* |@@SL.Debug_Task_RTInfo@@--------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,16)==14),225:226)'],1,[]);
Task=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.Debug_Task_RTInfo.Task = Task; % create struct
temp = reshape([data(find(mod(Count,16)==14),227:228)'],1,[]);
Payload=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.Debug_Task_RTInfo.Payload = Payload; % create struct
temp = reshape([data(find(mod(Count,16)==10),227:227)'],1,[]);
GSCmd=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_Task_RTInfo.GSCmd = GSCmd; % create struct
temp = reshape([data(find(mod(Count,16)==10),228:228)'],1,[]);
Warning=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_Task_RTInfo.Warning = Warning; % create struct
temp = reshape([data(find(mod(Count,16)==9),231:231)'],1,[]);
ComStatus=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_Task_RTInfo.ComStatus = ComStatus; % create struct
temp = reshape([data(find(mod(Count,16)==9),232:232)'],1,[]);
FenseStatus=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_Task_RTInfo.FenseStatus = FenseStatus; % create struct
temp = reshape([data(find(mod(Count,16)==12),231:231)'],1,[]);
StallStatus=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_Task_RTInfo.StallStatus = StallStatus; % create struct
temp = reshape([data(find(mod(Count,16)==12),232:232)'],1,[]);
SensorStatus=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_Task_RTInfo.SensorStatus = SensorStatus; % create struct
temp = reshape([data(find(mod(Count,16)==9),235:235)'],1,[]);
BatteryStatus=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_Task_RTInfo.BatteryStatus = BatteryStatus; % create struct
temp = reshape([data(find(mod(Count,16)==9),236:236)'],1,[]);
FixWingHeightStatus=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_Task_RTInfo.FixWingHeightStatus = FixWingHeightStatus; % create struct
temp = reshape([data(find(mod(Count,16)==0),243:243)'],1,[]);
FindWind=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_Task_RTInfo.FindWind = FindWind; % create struct
temp = reshape([data(find(mod(Count,16)==0),244:244)'],1,[]);
LandCond1_Acc_H=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_Task_RTInfo.LandCond1_Acc_H = LandCond1_Acc_H; % create struct
temp = reshape([data(find(mod(Count,16)==1),243:243)'],1,[]);
LandCond1_Vd_H=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_Task_RTInfo.LandCond1_Vd_H = LandCond1_Vd_H; % create struct
temp = reshape([data(find(mod(Count,16)==1),244:244)'],1,[]);
LandCond3_near=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_Task_RTInfo.LandCond3_near = LandCond3_near; % create struct
temp = reshape([data(find(mod(Count,16)==2),243:244)'],1,[]);
maxDist_Path2Home=double(typecast(uint8(temp),'int16')')/32768*32000.0000000000;
SL.Debug_Task_RTInfo.maxDist_Path2Home = maxDist_Path2Home; % create struct
temp = reshape([data(find(mod(Count,16)==3),243:244)'],1,[]);
realtimeFenseDist=double(typecast(uint8(temp),'int16')')/32768*32000.0000000000;
SL.Debug_Task_RTInfo.realtimeFenseDist = realtimeFenseDist; % create struct
% /* +===========+================================+=============+============+==============+ */
% /*-----------------------Debug_WindParam    ----------------------------------------------| */	
% /* |@@SL.Debug_WindParam@@----------------------+-------------+------------+--------------| */20200322
temp = reshape([data(find(mod(Count,16)==13),235:236)'],1,[]);
sailWindSpeed=double(typecast(uint8(temp),'int16')')/32768*50.0000000000;
SL.Debug_WindParam.sailWindSpeed = sailWindSpeed; % create struct
temp = reshape([data(find(mod(Count,16)==14),233:234)'],1,[]);
sailWindHeading=double(typecast(uint8(temp),'int16')')/32768*10.0000000000;
SL.Debug_WindParam.sailWindHeading = sailWindHeading; % create struct
temp = reshape([data(find(mod(Count,16)==14),235:236)'],1,[]);
windSpeedMax=double(typecast(uint8(temp),'int16')')/32768*50.0000000000;
SL.Debug_WindParam.windSpeedMax = windSpeedMax; % create struct
temp = reshape([data(find(mod(Count,16)==15),233:234)'],1,[]);
windSpeedMin=double(typecast(uint8(temp),'int16')')/32768*50.0000000000;
SL.Debug_WindParam.windSpeedMin = windSpeedMin; % create struct
temp = reshape([data(find(mod(Count,16)==15),235:236)'],1,[]);
maxWindHeading=double(typecast(uint8(temp),'int16')')/32768*10.0000000000;
SL.Debug_WindParam.maxWindHeading = maxWindHeading; % create struct
% /* +===========+================================+=============+============+==============+ */
% /*-----------------------GlobalWindEst      ----------------------------------------------| */	
% /* |@@SL.GlobalWindEst@@------------------------+-------------+------------+--------------| */20200602
temp = reshape([data(find(mod(Count,16)==0),246:246)'],1,[]);
oneCircleComplete=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.GlobalWindEst.oneCircleComplete = oneCircleComplete; % create struct
temp = reshape([data(find(mod(Count,16)==3),245:246)'],1,[]);
windSpeed_ms=double(typecast(uint8(temp),'int16')')/32768*50.0000000000;
SL.GlobalWindEst.windSpeed_ms = windSpeed_ms; % create struct
temp = reshape([data(find(mod(Count,16)==12),245:246)'],1,[]);
windHeading_rad=double(typecast(uint8(temp),'int16')')/32768*6.3000000000;
SL.GlobalWindEst.windHeading_rad = windHeading_rad; % create struct
% /* +===========+================================+=============+============+==============+ */
% /*-----------------------Debug_TaskLogData  ----------------------------------------------| */	
% /* |@@SL.Debug_TaskLogData@@--------------------+-------------+------------+--------------| */20200605
temp = reshape([data(find(mod(Count,4)==2),109:112)'],1,[]);
time_sec=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.Debug_TaskLogData.time_sec = time_sec; % create struct
temp = reshape([data(find(mod(Count,4)==2),113:113)'],1,[]);
blockName=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Debug_TaskLogData.blockName = blockName; % create struct
temp = reshape([data(find(mod(Count,4)==2),114:114)'],1,[]);
idx=double(typecast(uint8(temp),'int8')')/1*1.0000000000;
SL.Debug_TaskLogData.idx = idx; % create struct
temp = reshape([data(find(mod(Count,4)==2),115:116)'],1,[]);
message=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.Debug_TaskLogData.message = message; % create struct
temp = reshape([data(find(mod(Count,4)==2),117:120)'],1,[]);
var10=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
SL.Debug_TaskLogData.var10 = var10; % create struct
temp = reshape([data(find(mod(Count,4)==2),121:124)'],1,[]);
var11=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
SL.Debug_TaskLogData.var11 = var11; % create struct
temp = reshape([data(find(mod(Count,4)==2),125:128)'],1,[]);
var12=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
SL.Debug_TaskLogData.var12 = var12; % create struct
temp = reshape([data(find(mod(Count,4)==2),129:132)'],1,[]);
var13=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
SL.Debug_TaskLogData.var13 = var13; % create struct
temp = reshape([data(find(mod(Count,4)==2),133:136)'],1,[]);
var14=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
SL.Debug_TaskLogData.var14 = var14; % create struct
% /* +===========+================================+=============+============+==============+ */
% /* ------------------------------algo sl output -------------------------------------------| */
% /* ------------+RefModel_SystemArchitecture_Y.OUT_TASKMODE---------------------------------| */
% /* |@@SL.OUT_TASKMODE@@+--------------------------------+-------------+------------+-------| */
temp = reshape([data(find(mod(Count,16)==0),229:230)'],1,[]);
currentPointNum=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.OUT_TASKMODE.currentPointNum = currentPointNum; % create struct
temp = reshape([data(find(mod(Count,16)==0),231:232)'],1,[]);
prePointNum=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.OUT_TASKMODE.prePointNum = prePointNum; % create struct
temp = reshape([data(find(mod(Count,16)==1),229:230)'],1,[]);
validPathNum=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.OUT_TASKMODE.validPathNum = validPathNum; % create struct
temp = reshape([data(find(mod(Count,16)==1),231:232)'],1,[]);
headingCmd=double(typecast(uint8(temp),'int16')')/32768*6.3000000000;
SL.OUT_TASKMODE.headingCmd = headingCmd; % create struct
temp = reshape([data(find(mod(Count,16)==2),229:230)'],1,[]);
distToGo=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKMODE.distToGo = distToGo; % create struct
temp = reshape([data(find(mod(Count,16)==2),231:232)'],1,[]);
dz=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKMODE.dz = dz; % create struct
temp = reshape([data(find(mod(Count,16)==3),229:230)'],1,[]);
groundspeedCmd=double(typecast(uint8(temp),'int16')')/32768*500.0000000000;
SL.OUT_TASKMODE.groundspeedCmd = groundspeedCmd; % create struct
temp = reshape([data(find(mod(Count,16)==3),231:232)'],1,[]);
rollCmd=double(typecast(uint8(temp),'int16')')/32768*500.0000000000;
SL.OUT_TASKMODE.rollCmd = rollCmd; % create struct
temp = reshape([data(find(mod(Count,16)==4),229:230)'],1,[]);
turnRadiusCmd=double(typecast(uint8(temp),'int16')')/32768*500.0000000000;
SL.OUT_TASKMODE.turnRadiusCmd = turnRadiusCmd; % create struct
temp = reshape([data(find(mod(Count,16)==4),231:232)'],1,[]);
heightCmd=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKMODE.heightCmd = heightCmd; % create struct
temp = reshape([data(find(mod(Count,16)==5),229:232)'],1,[]);
turnCenterLL0=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
SL.OUT_TASKMODE.turnCenterLL0 = turnCenterLL0; % create struct
temp = reshape([data(find(mod(Count,16)==6),229:232)'],1,[]);
turnCenterLL1=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
SL.OUT_TASKMODE.turnCenterLL1 = turnCenterLL1; % create struct
temp = reshape([data(find(mod(Count,16)==7),229:230)'],1,[]);
dR_turn=double(typecast(uint8(temp),'int16')')/32768*500.0000000000;
SL.OUT_TASKMODE.dR_turn = dR_turn; % create struct
temp = reshape([data(find(mod(Count,16)==7),231:231)'],1,[]);
uavMode=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_TASKMODE.uavMode = uavMode; % create struct
temp = reshape([data(find(mod(Count,16)==7),232:232)'],1,[]);
flightTaskMode=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_TASKMODE.flightTaskMode = flightTaskMode; % create struct
temp = reshape([data(find(mod(Count,16)==8),229:229)'],1,[]);
flightControlMode=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_TASKMODE.flightControlMode = flightControlMode; % create struct
temp = reshape([data(find(mod(Count,16)==8),230:230)'],1,[]);
AutoManualMode=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_TASKMODE.AutoManualMode = AutoManualMode; % create struct
temp = reshape([data(find(mod(Count,16)==8),231:231)'],1,[]);
comStatus=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_TASKMODE.comStatus = comStatus; % create struct
temp = reshape([data(find(mod(Count,16)==9),229:230)'],1,[]);
maxClimbSpeed=double(typecast(uint8(temp),'int16')')/32768*100.0000000000;
SL.OUT_TASKMODE.maxClimbSpeed = maxClimbSpeed; % create struct
temp = reshape([data(find(mod(Count,16)==10),229:232)'],1,[]);
prePathPoint_LLA0=double(typecast(uint8(temp),'int32')')/10000000*10000.0000000000;
SL.OUT_TASKMODE.prePathPoint_LLA0 = prePathPoint_LLA0; % create struct
temp = reshape([data(find(mod(Count,16)==11),229:232)'],1,[]);
prePathPoint_LLA1=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
SL.OUT_TASKMODE.prePathPoint_LLA1 = prePathPoint_LLA1; % create struct
temp = reshape([data(find(mod(Count,16)==12),229:230)'],1,[]);
prePathPoint_LLA2=double(typecast(uint8(temp),'int16')')/32768*1.0000000000;
SL.OUT_TASKMODE.prePathPoint_LLA2 = prePathPoint_LLA2; % create struct
temp = reshape([data(find(mod(Count,16)==13),229:232)'],1,[]);
curPathPoint_LLA0=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
SL.OUT_TASKMODE.curPathPoint_LLA0 = curPathPoint_LLA0; % create struct
temp = reshape([data(find(mod(Count,16)==14),229:232)'],1,[]);
curPathPoint_LLA1=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
SL.OUT_TASKMODE.curPathPoint_LLA1 = curPathPoint_LLA1; % create struct
temp = reshape([data(find(mod(Count,16)==15),229:230)'],1,[]);
curPathPoint_LLA2=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKMODE.curPathPoint_LLA2 = curPathPoint_LLA2; % create struct
temp = reshape([data(find(mod(Count,16)==1),245:245)'],1,[]);
isTaskComplete=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_TASKMODE.isTaskComplete = isTaskComplete; % create struct
temp = reshape([data(find(mod(Count,16)==1),246:246)'],1,[]);
isHeadingRotate_OnGround=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_TASKMODE.isHeadingRotate_OnGround = isHeadingRotate_OnGround; % create struct
temp = reshape([data(find(mod(Count,16)==2),245:246)'],1,[]);
numTakeOff=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.OUT_TASKMODE.numTakeOff = numTakeOff; % create struct
temp = reshape([data(find(mod(Count,16)==8),232:232)'],1,[]);
isAllowedToPause=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_TASKMODE.isAllowedToPause = isAllowedToPause; % create struct
temp = reshape([data(find(mod(Count,16)==11),291:292)'],1,[]);
lastTargetPathPoint=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.OUT_TASKMODE.lastTargetPathPoint = lastTargetPathPoint; % create struct
temp = reshape([data(find(mod(Count,16)==12),289:292)'],1,[]);
LLATaskInterrupt0=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.OUT_TASKMODE.LLATaskInterrupt0 = LLATaskInterrupt0; % create struct
temp = reshape([data(find(mod(Count,16)==13),289:292)'],1,[]);
LLATaskInterrupt1=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.OUT_TASKMODE.LLATaskInterrupt1 = LLATaskInterrupt1; % create struct
temp = reshape([data(find(mod(Count,16)==14),289:292)'],1,[]);
LLATaskInterrupt2=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.OUT_TASKMODE.LLATaskInterrupt2 = LLATaskInterrupt2; % create struct
temp = reshape([data(find(mod(Count,16)==11),243:244)'],1,[]);
airspeedCmd=double(typecast(uint8(temp),'int16')')/32768*500.0000000000;
SL.OUT_TASKMODE.airspeedCmd = airspeedCmd; % create struct
% /* ------------+RefModel_SystemArchitecture_Y.OUT_TASKFLIGHTPARAM--------------------------| */
% /* |@@SL.OUT_TASKFLIGHTPARAM@@+-------------------------------+------------+---------------| */
temp = reshape([data(find(mod(Count,16)==0),233:236)'],1,[]);
curHomeLLA0=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
SL.OUT_TASKFLIGHTPARAM.curHomeLLA0 = curHomeLLA0; % create struct
temp = reshape([data(find(mod(Count,16)==1),233:236)'],1,[]);
curHomeLLA1=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
SL.OUT_TASKFLIGHTPARAM.curHomeLLA1 = curHomeLLA1; % create struct
temp = reshape([data(find(mod(Count,16)==2),233:234)'],1,[]);
curHomeLLA2=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKFLIGHTPARAM.curHomeLLA2 = curHomeLLA2; % create struct
temp = reshape([data(find(mod(Count,16)==2),235:236)'],1,[]);
curVelNED0=double(typecast(uint8(temp),'int16')')/32768*50.0000000000;
SL.OUT_TASKFLIGHTPARAM.curVelNED0 = curVelNED0; % create struct
temp = reshape([data(find(mod(Count,16)==3),233:234)'],1,[]);
curVelNED1=double(typecast(uint8(temp),'int16')')/32768*50.0000000000;
SL.OUT_TASKFLIGHTPARAM.curVelNED1 = curVelNED1; % create struct
temp = reshape([data(find(mod(Count,16)==3),235:236)'],1,[]);
curVelNED2=double(typecast(uint8(temp),'int16')')/32768*30.0000000000;
SL.OUT_TASKFLIGHTPARAM.curVelNED2 = curVelNED2; % create struct
temp = reshape([data(find(mod(Count,16)==4),233:234)'],1,[]);
curSpeed=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKFLIGHTPARAM.curSpeed = curSpeed; % create struct
temp = reshape([data(find(mod(Count,16)==4),235:236)'],1,[]);
curAirSpeed=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKFLIGHTPARAM.curAirSpeed = curAirSpeed; % create struct
temp = reshape([data(find(mod(Count,16)==5),233:234)'],1,[]);
curEuler0=double(typecast(uint8(temp),'int16')')/32768*10.0000000000;
SL.OUT_TASKFLIGHTPARAM.curEuler0 = curEuler0; % create struct
temp = reshape([data(find(mod(Count,16)==5),235:236)'],1,[]);
curEuler1=double(typecast(uint8(temp),'int16')')/32768*10.0000000000;
SL.OUT_TASKFLIGHTPARAM.curEuler1 = curEuler1; % create struct
temp = reshape([data(find(mod(Count,16)==6),233:234)'],1,[]);
curEuler2=double(typecast(uint8(temp),'int16')')/32768*10.0000000000;
SL.OUT_TASKFLIGHTPARAM.curEuler2 = curEuler2; % create struct
temp = reshape([data(find(mod(Count,16)==6),235:236)'],1,[]);
curWB0=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKFLIGHTPARAM.curWB0 = curWB0; % create struct
temp = reshape([data(find(mod(Count,16)==7),233:234)'],1,[]);
curWB1=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKFLIGHTPARAM.curWB1 = curWB1; % create struct
temp = reshape([data(find(mod(Count,16)==7),235:236)'],1,[]);
curWB2=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKFLIGHTPARAM.curWB2 = curWB2; % create struct
temp = reshape([data(find(mod(Count,16)==8),233:234)'],1,[]);
curPosNED0=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKFLIGHTPARAM.curPosNED0 = curPosNED0; % create struct
temp = reshape([data(find(mod(Count,16)==8),235:236)'],1,[]);
curPosNED1=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKFLIGHTPARAM.curPosNED1 = curPosNED1; % create struct
temp = reshape([data(find(mod(Count,16)==9),233:234)'],1,[]);
curPosNED2=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKFLIGHTPARAM.curPosNED2 = curPosNED2; % create struct
temp = reshape([data(find(mod(Count,16)==10),233:236)'],1,[]);
curLLA0=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
SL.OUT_TASKFLIGHTPARAM.curLLA0 = curLLA0; % create struct
temp = reshape([data(find(mod(Count,16)==11),233:236)'],1,[]);
curLLA1=double(typecast(uint8(temp),'int32')')/10000000*1.0000000000;
SL.OUT_TASKFLIGHTPARAM.curLLA1 = curLLA1; % create struct
temp = reshape([data(find(mod(Count,16)==12),233:234)'],1,[]);
curLLA2=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKFLIGHTPARAM.curLLA2 = curLLA2; % create struct
temp = reshape([data(find(mod(Count,16)==12),235:236)'],1,[]);
curGroundSpeed=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKFLIGHTPARAM.curGroundSpeed = curGroundSpeed; % create struct
temp = reshape([data(find(mod(Count,16)==13),233:234)'],1,[]);
curAccZ=double(typecast(uint8(temp),'int16')')/32768*10000.0000000000;
SL.OUT_TASKFLIGHTPARAM.curAccZ = curAccZ; % create struct
temp = reshape([data(find(mod(Count,4)==3),257:260)'],1,[]);
groundHomeLLA2=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.OUT_TASKFLIGHTPARAM.groundHomeLLA2 = groundHomeLLA2; % create struct
temp = reshape([data(find(mod(Count,4)==3),261:264)'],1,[]);
curHeightForControl=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.OUT_TASKFLIGHTPARAM.curHeightForControl = curHeightForControl; % create struct
% /* ------------+RefModel_SystemArchitecture_Y.OUT_NAVI2FIRM--------------------------------| */
% /* |@@SL.OUT_NAVI2FIRM@@+-----------------------+-------------+------------+---------------| */
temp = reshape([data(find(mod(Count,16)==0),245:245)'],1,[]);
isNavFilterGood=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_NAVI2FIRM.isNavFilterGood = isNavFilterGood; % create struct
% /* ------------+RefModel_SystemArchitecture_Y.OUT_SYSTEMINFO-------------------------------| */20200817
% /* |@@SL.OUT_SYSTEMINFO@@+----------------------+-------------+------------+---------------| */
temp = reshape([data(find(mod(Count,16)==15),291:291)'],1,[]);
uavModel	=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_SYSTEMINFO.uavModel	 = uavModel	; % create struct
% /* ------------+RefModel_SystemArchitecture_Y.OUT_FLIGHTPERF-------------------------------| */
% /* |@@SL.OUT_FLIGHTPERF@@+----------------------+-------------+------------+---------------| */
temp = reshape([data(find(mod(Count,8)==5),197:197)'],1,[]);
isAbleToCompleteTask=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.isAbleToCompleteTask = isAbleToCompleteTask; % create struct
temp = reshape([data(find(mod(Count,8)==5),198:198)'],1,[]);
flagGoHomeNow=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.flagGoHomeNow = flagGoHomeNow; % create struct
temp = reshape([data(find(mod(Count,8)==5),199:200)'],1,[]);
remainDistToGo_m=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.remainDistToGo_m = remainDistToGo_m; % create struct
temp = reshape([data(find(mod(Count,8)==5),201:202)'],1,[]);
remainTimeToSpend_sec=double(typecast(uint8(temp),'int16')')/32768*7200.0000000000;
SL.OUT_FLIGHTPERF.remainTimeToSpend_sec = remainTimeToSpend_sec; % create struct
temp = reshape([data(find(mod(Count,8)==5),203:204)'],1,[]);
remainPowerWhenFinish_per=double(typecast(uint8(temp),'int16')')/32768*100.0000000000;
SL.OUT_FLIGHTPERF.remainPowerWhenFinish_per = remainPowerWhenFinish_per; % create struct
temp = reshape([data(find(mod(Count,8)==5),205:206)'],1,[]);
economicAirspeed=double(typecast(uint8(temp),'int16')')/32768*100.0000000000;
SL.OUT_FLIGHTPERF.economicAirspeed = economicAirspeed; % create struct
temp = reshape([data(find(mod(Count,8)==0),293:294)'],1,[]);
remainPathPoint=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.remainPathPoint = remainPathPoint; % create struct
temp = reshape([data(find(mod(Count,8)==1),293:294)'],1,[]);
batteryLifeToCompleteTask=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.batteryLifeToCompleteTask = batteryLifeToCompleteTask; % create struct
temp = reshape([data(find(mod(Count,8)==2),293:294)'],1,[]);
batterylifeNeededToHome=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.batterylifeNeededToHome = batterylifeNeededToHome; % create struct
temp = reshape([data(find(mod(Count,16)==0),245:245)'],1,[]);
isNavFilterGood=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.isNavFilterGood = isNavFilterGood; % create struct
temp = reshape([data(find(mod(Count,16)==15),289:290)'],1,[]);
batterylifeNeededToLand=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.batterylifeNeededToLand = batterylifeNeededToLand; % create struct
% /* ------------+Debug_GroundStationShow----------------------------------------------------| */20200720
% /* |@@SL.Debug_GroundStationShow@@+-------------+-------------+------------+---------------| */20200720
temp = reshape([data(find(mod(Count,4)==3),221:222)'],1,[]);
windSpeed_ms=double(typecast(uint8(temp),'int16')')/32768*30.0000000000;
SL.Debug_GroundStationShow.windSpeed_ms = windSpeed_ms; % create struct
temp = reshape([data(find(mod(Count,16)==11),221:222)'],1,[]);
groundSpeed_ms=double(typecast(uint8(temp),'int16')')/32768*50.0000000000;
SL.Debug_GroundStationShow.groundSpeed_ms = groundSpeed_ms; % create struct
% /* ------------+Simulation_TaskAlgoParam---------------------------------------------------| */20200817
% /* |@@SL.Simulation_TaskAlgoParam@@+------------+-------------+------------+---------------| */20200817
temp = reshape([data(find(mod(Count,8)==3),294:294)'],1,[]);
isHoverDownToCenter=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Simulation_TaskAlgoParam.isHoverDownToCenter = isHoverDownToCenter; % create struct
temp = reshape([data(find(mod(Count,8)==3),293:293)'],1,[]);
runSingleTaskMode=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Simulation_TaskAlgoParam.runSingleTaskMode = runSingleTaskMode; % create struct
% /* +===========+================================+=============+============+===============+ */
% /* +===========+================================+=============+============+===============+ */
% /* |-----------+--------------------------------+-------------+------------+--------------| */
temp = reshape([data(find(mod(Count,4)==0),239:242)'],1,[]);
Lon=double(typecast(uint8(temp),'int32')')/10000000*10000000.0000000000;
SL.Simulation_TaskAlgoParam.Lon = Lon; % create struct
temp = reshape([data(find(mod(Count,4)==0),257:260)'],1,[]);
Lat=double(typecast(uint8(temp),'int32')')/10000000*10000000.0000000000;
SL.Simulation_TaskAlgoParam.Lat = Lat; % create struct
temp = reshape([data(find(mod(Count,4)==0),261:264)'],1,[]);
height=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.Simulation_TaskAlgoParam.height = height; % create struct
temp = reshape([data(find(mod(Count,4)==0),265:268)'],1,[]);
velN=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.Simulation_TaskAlgoParam.velN = velN; % create struct
temp = reshape([data(find(mod(Count,4)==0),269:272)'],1,[]);
velE=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.Simulation_TaskAlgoParam.velE = velE; % create struct
temp = reshape([data(find(mod(Count,4)==0),273:276)'],1,[]);
velD=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.Simulation_TaskAlgoParam.velD = velD; % create struct
temp = reshape([data(find(mod(Count,4)==0),277:280)'],1,[]);
delta_lon=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.Simulation_TaskAlgoParam.delta_lon = delta_lon; % create struct
temp = reshape([data(find(mod(Count,4)==0),281:284)'],1,[]);
delta_lat=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.Simulation_TaskAlgoParam.delta_lat = delta_lat; % create struct
temp = reshape([data(find(mod(Count,4)==0),285:288)'],1,[]);
delta_height=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.Simulation_TaskAlgoParam.delta_height = delta_height; % create struct
temp = reshape([data(find(mod(Count,4)==0),237:238)'],1,[]);
pDop=double(typecast(uint8(temp),'int16')')/32768*100.0000000000;
SL.Simulation_TaskAlgoParam.pDop = pDop; % create struct
temp = reshape([data(find(mod(Count,4)==0),39:39)'],1,[]);
BESTPOS=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Simulation_TaskAlgoParam.BESTPOS = BESTPOS; % create struct
temp = reshape([data(find(mod(Count,4)==0),40:40)'],1,[]);
numSv=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.Simulation_TaskAlgoParam.numSv = numSv; % create struct
% /* +===========+================================+=============+============+==============+ */
