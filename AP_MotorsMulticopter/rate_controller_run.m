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

roll_in  = rate_target_to_motor_rollg( rate_target_ang_vel(1),gyro_x,limit_roll_pitch)+Test_w.roll_in;
pitch_in = rate_target_to_motor_pitchg(rate_target_ang_vel(2),gyro_y,limit_roll_pitch)+Test_w.pitch_in;
yaw_in   = rate_target_to_motor_yawg(  rate_target_ang_vel(3),gyro_z,limit_yaw)+Test_w.yaw_in;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

