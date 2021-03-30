function   set_alt_target_from_climb_rate_ff(  climb_rate_cms,   dt,   force_descend)
 
%     set_alt_target_from_climb_rate_ff - adjusts target up or down using a climb rate in cm/s using feed-forward
%     should be called continuously (with dt set to be the expected time between calls)
%     actual position target will be moved no faster than the speed_down and speed_up
%     target will also be stopped if the motors hit their limits or leash length is exceeded
%     set force_descend to true during landing to allow target to move low enough to slow the motors


global AC_PosControl
global AP_Motors

speed_up_cms                  = AC_PosControl.speed_up_cms;
speed_down_cms                = AC_PosControl.speed_down_cms;
vel_desired                   = AC_PosControl.vel_desired;
pos_target                    = AC_PosControl.pos_target;
limit_pos_up                  = AC_PosControl.limit_pos_up;
POSCONTROL_JERK_RATIO         = AC_PosControl.POSCONTROL_JERK_RATIO;
accel_last_z_cms              = AC_PosControl.accel_last_z_cms;
flags_use_desvel_ff_z         = AC_PosControl.flags_use_desvel_ff_z;
POSCONTROL_OVERSPEED_GAIN_Z   = AC_PosControl.POSCONTROL_OVERSPEED_GAIN_Z;
limit_throttle_lower          = AP_Motors.limit_throttle_lower;
limit_throttle_upper          = AP_Motors.limit_throttle_upper;

    % calculated increased maximum acceleration if over speed
      accel_z_cms = AC_PosControl.accel_z_cms;
    if (vel_desired(3) < speed_down_cms && speed_down_cms~=0)  
        accel_z_cms =accel_z_cms*POSCONTROL_OVERSPEED_GAIN_Z * vel_desired(3) / speed_down_cms;
    end
    if (vel_desired(3) > speed_up_cms &&speed_up_cms~=0)  
        accel_z_cms= accel_z_cms* POSCONTROL_OVERSPEED_GAIN_Z * vel_desired(3) / speed_up_cms;
    end
    accel_z_cms = constrain_value(accel_z_cms, 0.0, 750.0);

    % jerk_z is calculated to reach full acceleration in 1000ms.
      jerk_z = accel_z_cms * POSCONTROL_JERK_RATIO;
      accel_z_max = min(accel_z_cms, sqrt(2.0 * abs(vel_desired(3) - climb_rate_cms) * jerk_z));

    accel_last_z_cms=accel_last_z_cms + jerk_z * dt;
    accel_last_z_cms = min(accel_z_max, accel_last_z_cms);

     vel_change_limit = accel_last_z_cms * dt;
    vel_desired(3) = constrain_value(climb_rate_cms, vel_desired(3) - vel_change_limit, vel_desired(3) + vel_change_limit);
    flags_use_desvel_ff_z = true;

    % adjust desired alt if motors have not hit their limits
    % To-Do: add check of _limit.pos_down?
    if ((vel_desired(3) < 0 && (~limit_throttle_lower || force_descend)) || (vel_desired(3) > 0 && ~limit_throttle_upper && ~limit_pos_up))  
        pos_target(3) =pos_target(3)+vel_desired(3) * dt;
    end
     
AC_PosControl.speed_up_cms                  = speed_up_cms;
AC_PosControl.speed_down_cms                = speed_down_cms;
AC_PosControl.vel_desired                   = vel_desired;
AC_PosControl.pos_target                    = pos_target;
AC_PosControl.limit_pos_up                  = limit_pos_up;
AC_PosControl.POSCONTROL_JERK_RATIO         = POSCONTROL_JERK_RATIO;
AC_PosControl.accel_last_z_cms              = accel_last_z_cms;
AC_PosControl.flags_use_desvel_ff_z         = flags_use_desvel_ff_z;
AC_PosControl.POSCONTROL_OVERSPEED_GAIN_Z   = POSCONTROL_OVERSPEED_GAIN_Z;
AP_Motors.limit_throttle_lower              = limit_throttle_lower;
AP_Motors.limit_throttle_upper              = limit_throttle_upper;
  
  
  
end

