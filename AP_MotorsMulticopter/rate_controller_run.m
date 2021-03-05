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
roll_in  = rate_target_to_motor_roll(  gyro_x,  rate_target_ang_vel(1))+Test_w.roll_in;
pitch_in = rate_target_to_motor_pitchg( gyro_y,  rate_target_ang_vel(2),limit_roll_pitch)+Test_w.pitch_in;
yaw_in   = rate_target_to_motor_yaw(  gyro_z,   rate_target_ang_vel(3))+Test_w.yaw_in;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

