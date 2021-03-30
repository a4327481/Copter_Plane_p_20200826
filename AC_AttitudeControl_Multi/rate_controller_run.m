function rate_controller_run()
global gyro_x
global gyro_y
global gyro_z
global Test_w
global AP_Motors
global AC_Attitude

 limit_roll                      = AP_Motors.limit_roll;
 limit_pitch                     = AP_Motors.limit_pitch;
 limit_yaw                       = AP_Motors.limit_yaw;
roll_in                 = AP_Motors.roll_in;
pitch_in                = AP_Motors.pitch_in;
yaw_in                  = AP_Motors.yaw_in;    
rate_target_ang_vel     = AC_Attitude.rate_target_ang_vel; 
% move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteratio
update_throttle_rpy_mix();

rate_target_ang_vel(1)=rate_target_ang_vel(1)+Test_w.rate_target_ang_vel0;
rate_target_ang_vel(2)=rate_target_ang_vel(2)+Test_w.rate_target_ang_vel1;
rate_target_ang_vel(3)=rate_target_ang_vel(3)+Test_w.rate_target_ang_vel2;


roll_in  = rate_target_to_motor_rollg( rate_target_ang_vel(1),gyro_x,limit_roll)+Test_w.roll_in;
pitch_in = rate_target_to_motor_pitchg(rate_target_ang_vel(2),gyro_y,limit_pitch)+Test_w.pitch_in;
yaw_in   = rate_target_to_motor_yawg(  rate_target_ang_vel(3),gyro_z,limit_yaw)+Test_w.yaw_in;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 AP_Motors.limit_roll                      = limit_roll;
 AP_Motors.limit_pitch                     = limit_pitch;
 AP_Motors.limit_yaw                       = limit_yaw ;
AP_Motors.roll_in                 = roll_in;
AP_Motors.pitch_in                = pitch_in;
AP_Motors.yaw_in                  = yaw_in;    
AC_Attitude.rate_target_ang_vel   = rate_target_ang_vel; 


end

