function output=rate_target_to_motor_yawg( target_in,  measurement,  limit)
global AC_rate_yaw_pid

[output ,AC_rate_yaw_pid]=AC_PID_update_all(target_in,  measurement,  limit,AC_rate_yaw_pid);
output =AC_rate_yaw_pid.kff*target_in+output;

end

