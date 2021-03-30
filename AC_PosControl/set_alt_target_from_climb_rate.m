function set_alt_target_from_climb_rate(  climb_rate_cms,   dt,   force_descend)
% set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
%     should be called continuously (with dt set to be the expected time between calls)
%     actual position target will be moved no faster than the speed_down and speed_up
%     target will also be stopped if the motors hit their limits or leash length is exceeded
 global limit_throttle_lower
 global limit_throttle_upper
 global limit_pos_up
 global flags_use_desvel_ff_z
 global vel_desired
 global pos_target
 
limit_throttle_lower                    = AP_Motors.limit_throttle_lower;
limit_throttle_upper                    = AP_Motors.limit_throttle_upper;
limit_pos_up                            = AC_PosControl.limit_pos_up;
flags_use_desvel_ff_z                   = AC_PosControl.flags_use_desvel_ff_z;
vel_desired                             = AC_PosControl.vel_desired;
pos_target                              = AC_PosControl.pos_target ;
 
    % adjust desired alt if motors have not hit their limits
    % To-Do: add check of _limit.pos_down?
    if ((climb_rate_cms < 0 && (~limit_throttle_lower || force_descend)) || (climb_rate_cms > 0 && ~limit_throttle_upper && ~limit_pos_up))  
         pos_target(3)=pos_target(3)+climb_rate_cms * dt;
    end

    % do not use z-axis desired velocity feed forward
    % vel_desired set to desired climb rate for reporting and land-detector
     flags_use_desvel_ff_z = 0;
     vel_desired(3) = climb_rate_cms;
 
AP_Motors.limit_throttle_lower                        = limit_throttle_lower;
AP_Motors.limit_throttle_upper                        = limit_throttle_upper;
AC_PosControl.limit_pos_up                            = limit_pos_up;
AC_PosControl.flags_use_desvel_ff_z                   = flags_use_desvel_ff_z;
AC_PosControl.vel_desired                             = vel_desired;
AC_PosControl.pos_target                              = pos_target ;



end

