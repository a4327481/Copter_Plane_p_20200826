%V10 mat load

out_filePath=data_read_V10();
V10Log = V10_decode_auto(out_filePath);
% time_algo=V10Log.BUS_CTRL.TimeUS*1e-4;
parserData = fieldnames(V10Log);
for i = 1:length(parserData)
    V10Log_parserData=V10Log.(parserData{i});
    V10Log_parserData_i=fieldnames(V10Log_parserData);
    V10Log_parserData=alignDimension_min(V10Log_parserData);
    
    if(strcmp(parserData{i,1},'BUS_CTRL'))
         assignin('base',['time_algo'],V10Log_parserData.(V10Log_parserData_i{1,1})*1e-4)
        for j=2:length(V10Log_parserData_i)
            len=size(V10Log_parserData.(V10Log_parserData_i{j,1}),2);
            if(len<=1)
                assignin('base',['algo_',V10Log_parserData_i{j}],V10Log_parserData.(V10Log_parserData_i{j,1}))
            else
                for jj=1:len
                    assignin('base',['algo_',V10Log_parserData_i{j},'_',num2str(jj-1)],V10Log_parserData.(V10Log_parserData_i{j,1})(:,jj))
                end
            end
        end
    end 
    
    if(strcmp(parserData{i,1},'IMU1_Driver'))
        assignin('base',['time_imuf'],V10Log_parserData.(V10Log_parserData_i{1,1})*1e-4)
        for j=2:length(V10Log_parserData_i)
            assignin('base',[V10Log_parserData_i{j},'_f'],V10Log_parserData.(V10Log_parserData_i{j,1}))
        end
    end
    if(strcmp(parserData{i,1},['OUT_TASKFLIGHTPARAM']))
        assignin('base',['time_sl_task_flight'],V10Log_parserData.(V10Log_parserData_i{1,1})*1e-4)
        for j=2:length(V10Log_parserData_i)
            len=size(V10Log_parserData.(V10Log_parserData_i{j,1}),2);
            if(len<=1)
                assignin('base',['algo_',V10Log_parserData_i{j}],V10Log_parserData.(V10Log_parserData_i{j,1}))
            else
                for jj=1:len
                    assignin('base',['algo_',V10Log_parserData_i{j},'_',num2str(jj-1)],V10Log_parserData.(V10Log_parserData_i{j,1})(:,jj))
                end
            end
        end
    end
    
    if(strcmp(parserData{i,1},['OUT_NAVI2CONTROL']))
        assignin('base',['time_sl_att'],V10Log_parserData.(V10Log_parserData_i{1,1})*1e-4)
        for j=2:length(V10Log_parserData_i)
            len=size(V10Log_parserData.(V10Log_parserData_i{j,1}),2);
            if(len<=1)
                assignin('base',['algo_',V10Log_parserData_i{j}],V10Log_parserData.(V10Log_parserData_i{j,1}))
            else
                for jj=1:len
                    assignin('base',['algo_',V10Log_parserData_i{j},'_',num2str(jj-1)],V10Log_parserData.(V10Log_parserData_i{j,1})(:,jj))
                end
            end
        end
    end
    
    if(strcmp(parserData{i,1},['AirSpeed2Wing']))
        assignin('base',['time_arspeed'],V10Log_parserData.(V10Log_parserData_i{1,1})*1e-4)
        for j=2:length(V10Log_parserData_i)
            len=size(V10Log_parserData.(V10Log_parserData_i{j,1}),2);
            if(len<=1)
                assignin('base',['algo_',V10Log_parserData_i{j}],V10Log_parserData.(V10Log_parserData_i{j,1}))
            else
                for jj=1:len
                    assignin('base',['algo_',V10Log_parserData_i{j},'_',num2str(jj-1)],V10Log_parserData.(V10Log_parserData_i{j,1})(:,jj))
                end
            end
        end
    end
    
    if(strcmp(parserData{i,1},['ALGO_DRIVER_STATE']))
        assignin('base',['time_state'],V10Log_parserData.(V10Log_parserData_i{1,1})*1e-4)
        for j=2:length(V10Log_parserData_i)
            len=size(V10Log_parserData.(V10Log_parserData_i{j,1}),2);
            if(len<=1)
                assignin('base',['algo_',V10Log_parserData_i{j}],V10Log_parserData.(V10Log_parserData_i{j,1}))
            else
                for jj=1:len
                    assignin('base',['algo_',V10Log_parserData_i{j},'_',num2str(jj-1)],V10Log_parserData.(V10Log_parserData_i{j,1})(:,jj))
                end
            end
        end
    end    
    
    if(strcmp(parserData{i,1},['OUT_TASKMODE']))
        assignin('base',['time_sl_taskmode'],V10Log_parserData.(V10Log_parserData_i{1,1})*1e-4)
        for j=2:length(V10Log_parserData_i)
            len=size(V10Log_parserData.(V10Log_parserData_i{j,1}),2);
            if(len<=1)
                assignin('base',['algo_',V10Log_parserData_i{j}],V10Log_parserData.(V10Log_parserData_i{j,1}))
            else
                for jj=1:len
                    assignin('base',['algo_',V10Log_parserData_i{j},'_',num2str(jj-1)],V10Log_parserData.(V10Log_parserData_i{j,1})(:,jj))
                end
            end
        end
    end
    
    
    if(strcmp(parserData{i,1},['GPS_Raw']))
        assignin('base',['time_gps'],V10Log_parserData.(V10Log_parserData_i{1,1})*1e-4)
        for j=2:length(V10Log_parserData_i)
            len=size(V10Log_parserData.(V10Log_parserData_i{j,1}),2);
            if(len<=1)
                assignin('base',['gps_',V10Log_parserData_i{j}],V10Log_parserData.(V10Log_parserData_i{j,1}))
            else
                for jj=1:len
                    assignin('base',['gps_',V10Log_parserData_i{j},'_',num2str(jj-1)],V10Log_parserData.(V10Log_parserData_i{j,1})(:,jj))
                end
            end
        end
    end
end
HD=180/pi;
algo_nav_roll=cosd(algo_pitch).*atan(algo_latAccDem * 0.101972) *HD;
algo_nav_pitch=algo_nav_pitch_cd/100;
algo_curr_pos_0=algo_curr_pos_0/100;
algo_curr_pos_1=algo_curr_pos_1/100;
algo_vel_target_0=algo_vel_target_0/100;
algo_vel_target_1=algo_vel_target_1/100;
algo_vel_target_2=algo_vel_target_2/100;
algo_vel_desired_2=algo_vel_desired_2/100;
algo_accel_target_0=algo_accel_target_0/100;
algo_accel_target_1=algo_accel_target_1/100;
algo_accel_target_2=algo_accel_target_2/100;
algo_z_accel_meas=algo_z_accel_meas/100;
algo_climb_rate_cms=algo_climb_rate_cms/100;
algo_roll_target=algo_roll_target/100;
algo_pitch_target=algo_pitch_target/100;
gx_f=interp1(time_imuf,gx_f,time_algo)*HD;
gy_f=interp1(time_imuf,gy_f,time_algo)*HD;
gz_f=interp1(time_imuf,gz_f,time_algo)*HD;
ax_f=interp1(time_imuf,ax_f,time_algo);
ay_f=interp1(time_imuf,ay_f,time_algo);
az_f=interp1(time_imuf,az_f,time_algo);


algo_roll=algo_roll*HD;
algo_pitch=algo_pitch*HD;
algo_yaw=algo_yaw*HD;
algo_curr_vel_0=algo_curr_vel_0/100;
algo_curr_vel_1=algo_curr_vel_1/100;
algo_curr_vel_2=algo_curr_vel_2/100;
algo_pos_target_0=algo_pos_target_0/100;
algo_pos_target_1=algo_pos_target_1/100;
algo_pos_target_2=algo_pos_target_2/100;
algo_attitude_target_euler_angle_0=algo_attitude_target_euler_angle_0*HD;
algo_attitude_target_euler_angle_1=algo_attitude_target_euler_angle_1*HD;
algo_attitude_target_euler_angle_2=algo_attitude_target_euler_angle_2*HD;
algo_rate_target_ang_vel_0=algo_rate_target_ang_vel_0*HD;
algo_rate_target_ang_vel_1=algo_rate_target_ang_vel_1*HD;
algo_rate_target_ang_vel_2=algo_rate_target_ang_vel_2*HD;
algo_roll_in=algo_roll_in*10;
algo_pitch_in=algo_pitch_in*10;
algo_yaw_in=algo_yaw_in*10;
algo_throttle_filter=algo_throttle_filter*10;
algo_pitch_dem=algo_pitch_dem*HD;
algo_k_rudder=algo_k_rudder/100;
algo_k_aileron=algo_k_aileron/100;
algo_k_elevator=algo_k_elevator/100;
algo_k_throttle=algo_k_throttle*10;
algo_curr_alt=algo_curr_alt/100;
% algo_current_loc_0=interp1(V10Log.GPS_Raw.Time_100us*1e-4,V10Log.GPS_Raw.lat,time_algo);
% algo_current_loc_1=interp1(V10Log.GPS_Raw.Time_100us*1e-4,V10Log.GPS_Raw.lon,time_algo);
% algo_current_loc_0=interp1(V10Log.GPS_Raw.Time_100us*1e-4,V10Log.GPS_Raw.lat,time_algo);
% algo_current_loc_1=interp1(V10Log.GPS_Raw.Time_100us*1e-4,V10Log.GPS_Raw.lon,time_algo);
algo_current_loc_0=algo_current_loc_0*1e-7;
algo_current_loc_1=algo_current_loc_1*1e-7;

EAS2TAS_Algo =algo_EAS2TAS_Algo;
algo_arspeed=algo_airspeed;
algo_NAV_alt=algo_alt;
algo_remote_ct_st_pitch = interp1(time_state,algo_algo_remot_pitch,time_algo);
algo_remote_ct_st_roll  = interp1(time_state,algo_algo_remot_roll,time_algo);
algo_remote_ct_st_yaw   = interp1(time_state,algo_algo_remot_yaw,time_algo);
algo_remote_ct_st_throttle   = interp1(time_state,algo_algo_remot_throttle,time_algo);
algo_heightCmd_sl=algo_heightCmd;
algo_flightTaskMode_sl=algo_flightTaskMode;
algo_maxClimbSpeed_sl=algo_maxClimbSpeed;
algo_tail_tilt_pwm = algo_pwm_tail;
algo_pwm_out_4=algo_pwm_out_3;
algo_pwm_out_3=algo_pwm_out_2;
algo_pwm_out_2=algo_pwm_out_1;
algo_pwm_out_1=algo_pwm_out_0;
algo_plane_mode=interp1(time_algo,algo_plane_mode,time_sl_taskmode);
