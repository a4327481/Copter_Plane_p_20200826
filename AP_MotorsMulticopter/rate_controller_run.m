function rate_controller_run()
global rate_target_ang_vel
global gyro_x
global gyro_y
global gyro_z
global roll_in
global pitch_in
global yaw_in
roll_in=rate_target_to_motor_roll(  gyro_x,  rate_target_ang_vel(1));
pitch_in=rate_target_to_motor_pitch( gyro_y,  rate_target_ang_vel(2));
yaw_in=rate_target_to_motor_yaw(  gyro_z,   rate_target_ang_vel(3));
yaw_in=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% global dt
% global thrust_slew_time
% global roll_factor
% global pitch_factor
% global yaw_factor



% global pitch
% global roll
% global wx
% global wy
% global wz
% global is_angle_rate
% global HD
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% persistent is_angle
% persistent delay_angle
% persistent is_rate_p
% persistent delay_rate_p
% persistent is_rate_n
% persistent delay_rate_n
% 
%     if isempty(is_angle)
%         is_angle = 0;
%     end
%     if isempty(is_rate_p)
%         is_rate_p = 0;
%     end    
%     if isempty(is_rate_n)
%         is_rate_n = 0;
%     end  
% if(pitch>45/HD||roll>45/HD)
%     is_angle=1;
% end
% if(wx>250/HD||wy>250/HD)
%     is_rate_p=1;
% end
% if(wx<-250/HD||wy<-250/HD)
%     is_rate_n=1;
% end
% if(is_angle&&is_rate_p&&is_rate_n)
%     is_angle_rate=1;
% end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %scale
%     thrust_dt=dt/thrust_slew_time*2/3;
%     if(max(abs(roll_factor))~=0)
%         max_roll_factor=max(abs(roll_factor));
%     else
%         max_roll_factor=1;
%     end
%     if(max(abs(pitch_factor))~=0)
%         max_pitch_factor=max(abs(pitch_factor));
%     else
%         max_pitch_factor=1;
%     end
%     if(max(abs(yaw_factor))~=0)
%         max_yaw_factor=max(abs(yaw_factor));
%     else
%         max_yaw_factor=1;
%     end
%     max(abs(roll_factor))
% thrust_dt_roll=thrust_dt/max_roll_factor*0.4;
% thrust_dt_pitch=thrust_dt/max_pitch_factor*0.4;
% thrust_dt_yaw=thrust_dt/max_yaw_factor*0.2;  
% roll_in_temp=rate_target_to_motor_roll(  gyro_x,  rate_target_ang_vel(1));
% pitch_in_temp=rate_target_to_motor_pitch( gyro_y,  rate_target_ang_vel(2));
% yaw_in_temp=rate_target_to_motor_yaw(  gyro_z,   rate_target_ang_vel(3));  
% 
% roll_in =constrain_value(roll_in_temp ,roll_in-thrust_dt_roll,roll_in+thrust_dt_roll);
% pitch_in=constrain_value(pitch_in_temp,pitch_in-thrust_dt_pitch,pitch_in+thrust_dt_pitch);
% yaw_in  =constrain_value(yaw_in_temp  ,yaw_in-thrust_dt_yaw,yaw_in+thrust_dt_yaw);

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

