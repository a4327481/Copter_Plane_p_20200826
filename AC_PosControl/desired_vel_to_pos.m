function desired_vel_to_pos(  nav_dt)
%  desired_vel_to_pos - move position target using desired velocities
    % range check nav_dt
    global AC_PosControl
    flags_reset_desired_vel_to_pos           = AC_PosControl.flags_reset_desired_vel_to_pos;
    pos_target                         = AC_PosControl.pos_target;
    vel_desired                        = AC_PosControl.vel_desired;
    if (nav_dt < 0)  
        return;
    end
     
    % update target position
    if (flags_reset_desired_vel_to_pos)  
        flags_reset_desired_vel_to_pos = false;
      else  
        pos_target(1) = pos_target(1) + vel_desired(1) * nav_dt;
        pos_target(2) = pos_target(2) + vel_desired(2) * nav_dt;
    end
    AC_PosControl.flags_reset_desired_vel_to_pos          = flags_reset_desired_vel_to_pos;
    AC_PosControl.pos_target                        = pos_target;
    AC_PosControl.vel_desired                       = vel_desired;
end

