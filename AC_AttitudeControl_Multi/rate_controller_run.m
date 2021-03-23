function rate_controller_run()
global rate_target_ang_vel
global gyro_x
global gyro_y
global gyro_z
global roll_in
global pitch_in
global yaw_in
global Test_w
global limit_roll_pitch
global limit_yaw
    
% move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteratio
update_throttle_rpy_mix();

rate_target_ang_vel(1)=rate_target_ang_vel(1)+Test_w.rate_target_ang_vel0;
rate_target_ang_vel(2)=rate_target_ang_vel(2)+Test_w.rate_target_ang_vel1;
rate_target_ang_vel(3)=rate_target_ang_vel(3)+Test_w.rate_target_ang_vel2;


roll_in  = rate_target_to_motor_rollg( rate_target_ang_vel(1),gyro_x,limit_roll_pitch)+Test_w.roll_in;
pitch_in = rate_target_to_motor_pitchg(rate_target_ang_vel(2),gyro_y,limit_roll_pitch)+Test_w.pitch_in;
yaw_in   = rate_target_to_motor_yawg(  rate_target_ang_vel(3),gyro_z,limit_yaw)+Test_w.yaw_in;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

