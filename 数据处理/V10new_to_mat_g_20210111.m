%V1000 to txt  
% Display surf plot of the currently selected data.
%mat 格式转换为画图
% global PathName
% if PathName~=0
%     cd(PathName);
%     [FileName,PathName,~] = uigetfile([PathName,'\\*.*']); % 读取雷达数据
% else
%     [FileName,PathName,~] = uigetfile('*.*'); % 读取雷达数据
% end
% if FileName==0
%     return;
% end
HD=180/pi;
out_filePath=data_read_V10();
V10Log = V10_decode_auto(out_filePath);
V10Log.BUS_CTRL.roll           = (V10Log.BUS_CTRL.roll)*HD; 
V10Log.BUS_CTRL.pitch          = (V10Log.BUS_CTRL.pitch)*HD; 
V10Log.BUS_CTRL.yaw            = (V10Log.BUS_CTRL.yaw)*HD; 
V10Log.BUS_CTRL.nav_roll       = cosd(V10Log.BUS_CTRL.pitch).*atan(V10Log.BUS_CTRL.latAccDem * 0.101972) *HD;
V10Log.BUS_CTRL.nav_pitch      = V10Log.BUS_CTRL.nav_pitch_cd/100;
V10Log.BUS_CTRL.curr_pos       = V10Log.BUS_CTRL.curr_pos/100;
V10Log.BUS_CTRL.vel_target     = V10Log.BUS_CTRL.vel_target/100;
V10Log.BUS_CTRL.vel_desired    = V10Log.BUS_CTRL.vel_desired/100;
V10Log.BUS_CTRL.accel_target   = V10Log.BUS_CTRL.accel_target/100;
V10Log.BUS_CTRL.z_accel_meas   = V10Log.BUS_CTRL.z_accel_meas/100;
V10Log.BUS_CTRL.climb_rate_cms = V10Log.BUS_CTRL.climb_rate_cms/100;
V10Log.BUS_CTRL.roll_target    = V10Log.BUS_CTRL.roll_target/100;
V10Log.BUS_CTRL.pitch_target   = V10Log.BUS_CTRL.pitch_target/100;
V10Log.BUS_CTRL.curr_vel       = V10Log.BUS_CTRL.curr_vel/100;
V10Log.BUS_CTRL.pos_target     = V10Log.BUS_CTRL.pos_target/100;
V10Log.BUS_CTRL.attitude_target_euler_angle  =  V10Log.BUS_CTRL.attitude_target_euler_angle*HD;
V10Log.BUS_CTRL.rate_target_ang_vel          =  V10Log.BUS_CTRL.rate_target_ang_vel*HD;
V10Log.BUS_CTRL.roll_in=V10Log.BUS_CTRL.roll_in*10;
V10Log.BUS_CTRL.pitch_in=V10Log.BUS_CTRL.pitch_in*10;
V10Log.BUS_CTRL.yaw_in=V10Log.BUS_CTRL.yaw_in*10;
V10Log.BUS_CTRL.throttle_in=V10Log.BUS_CTRL.throttle_in*10;
V10Log.BUS_CTRL.throttle_filter=V10Log.BUS_CTRL.throttle_filter*10;
V10Log.BUS_CTRL.pitch_dem=V10Log.BUS_CTRL.pitch_dem*HD;
V10Log.BUS_CTRL.k_rudder=V10Log.BUS_CTRL.k_rudder/100;
V10Log.BUS_CTRL.k_aileron=V10Log.BUS_CTRL.k_aileron/100;
V10Log.BUS_CTRL.k_elevator=V10Log.BUS_CTRL.k_elevator/100;
V10Log.BUS_CTRL.k_throttle=V10Log.BUS_CTRL.k_throttle*10;
V10Log.BUS_CTRL.curr_alt=V10Log.BUS_CTRL.curr_alt/100;
V10Log.BUS_CTRL.current_loc =V10Log.BUS_CTRL.current_loc*1e-7;
V10Log.BUS_CTRL.curr_V      =  hypot(V10Log.BUS_CTRL.curr_vel(:,1),V10Log.BUS_CTRL.curr_vel(:,2));
V10Log.IMU1_Driver.gx           = V10Log.IMU1_Driver.gx*HD;
V10Log.IMU1_Driver.gy           = V10Log.IMU1_Driver.gy*HD;
V10Log.IMU1_Driver.gz           = V10Log.IMU1_Driver.gz*HD;

parserData = fieldnames(V10Log);
for i = 1:length(parserData)
    V10Log_parserData=V10Log.(parserData{i});
    V10Log_parserData_i=fieldnames(V10Log_parserData);
    V10Log_parserData=alignDimension_max(V10Log_parserData);
    
    data = V10Log_parserData.(V10Log_parserData_i{1,1});
    for j=2:length(V10Log_parserData_i)
        temp_data=V10Log_parserData.(V10Log_parserData_i{j,1});
        data=[ data temp_data];
    end
    
    head=[V10Log_parserData_i{1}];
    for j=2:length(V10Log_parserData_i)
        len=size(V10Log_parserData.(V10Log_parserData_i{j,1}),2);
        for jj=1:len
        head=[head ,' ',V10Log_parserData_i{j}];
        end
    end
    head=[head ,'\n'];
    data_ck=data(1:1:end,:);
    data_ck(:,1)=data_ck(:,1)/1e4;
    fid=fopen([out_filePath(1:end-4),'_',parserData{i},'.dat'],'w');
    fprintf(fid,head);
    fclose(fid);
    save([out_filePath(1:end-4),'_',parserData{i},'.dat'],'data_ck','-ascii','-append' )

% 	fprintf('output:		%s\n',parserData{i});
% 	assignin('base',parserData{i},V10Log.(parserData{i}));
end

% plot(ALD1(1:end-1,3)/1e4,diff(ALD1(:,3))/10)
