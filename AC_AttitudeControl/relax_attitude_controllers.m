function  relax_attitude_controllers()
%  Ensure attitude controller have zero errors to relax rate controller output
global SINS
global AC_Attitude

global AC_rate_roll_pid
global AC_rate_pitch_pid
global AC_rate_yaw_pid
global AC_PosControl
gyro_x                          = SINS.gyro_x;
gyro_y                          = SINS.gyro_y;
gyro_z                          = SINS.gyro_z;
rot_body_to_ned                 = SINS.rot_body_to_ned;
attitude_target_quat            = AC_Attitude.attitude_target_quat;
attitude_target_euler_angle     = AC_Attitude.attitude_target_euler_angle;
attitude_ang_error              = AC_Attitude.attitude_ang_error;
attitude_target_ang_vel         = AC_Attitude.attitude_target_ang_vel;
attitude_target_euler_rate      = AC_Attitude.attitude_target_euler_rate;
rate_target_ang_vel             = AC_Attitude.rate_target_ang_vel;
thrust_error_angle              = AC_Attitude.thrust_error_angle;


% Initialize the attitude variables to the current attitude
attitude_target_quat=from_rotation_matrix(rot_body_to_ned);
attitude_target_euler_angle=to_euler(attitude_target_quat);
attitude_ang_error=[1 0 0 0];

% Initialize the angular rate variables to the current rate
attitude_target_ang_vel = [gyro_x gyro_y gyro_z];
attitude_target_euler_rate = ang_vel_to_euler_rate(attitude_target_euler_angle,attitude_target_ang_vel);
rate_target_ang_vel = [gyro_x gyro_y gyro_z];

% Initialize remaining variables
thrust_error_angle = 0.0;

% Reset the PID filters
AC_rate_roll_pid.flags_reset_filter=true;
AC_rate_pitch_pid.flags_reset_filter=true;
AC_rate_yaw_pid.flags_reset_filter=true;

AC_Attitude.attitude_target_quat            = attitude_target_quat;
AC_Attitude.attitude_target_euler_angle     = attitude_target_euler_angle;
AC_Attitude.attitude_ang_error              = attitude_ang_error;
AC_Attitude.attitude_target_ang_vel         = attitude_target_ang_vel;
AC_Attitude.attitude_target_euler_rate      = attitude_target_euler_rate;
AC_Attitude.rate_target_ang_vel             = rate_target_ang_vel;
AC_Attitude.thrust_error_angle              = thrust_error_angle;
AC_PosControl.target_yaw_rate               = 0;

end

