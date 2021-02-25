function rate_controller_run()
global rate_target_ang_vel
global gyro_x
global gyro_y
global gyro_z
global roll_in
global pitch_in
global yaw_in
global Test_w
roll_in=rate_target_to_motor_roll(  gyro_x,  rate_target_ang_vel(1))+Test_w.roll_in;
pitch_in=rate_target_to_motor_pitch( gyro_y,  rate_target_ang_vel(2))+Test_w.pitch_in;
yaw_in=rate_target_to_motor_yaw(  gyro_z,   rate_target_ang_vel(3))+Test_w.yaw_in;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

