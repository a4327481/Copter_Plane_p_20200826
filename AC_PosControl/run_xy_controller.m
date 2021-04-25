function run_xy_controller(dt)

global GRAVITY_MSS
global HD
global AC_PosControl
global AP_Motors
global SINS

curr_pos                            = SINS.curr_pos;
curr_vel                            = SINS.curr_vel;
p_pos_xy                            = AC_PosControl.p_pos_xy;
accel_cms                           = AC_PosControl.accel_cms;
leash                               = AC_PosControl.leash;
accel_xy_filt_hz                    = AC_PosControl.accel_xy_filt_hz;
POSCONTROL_ACCEL_XY_MAX             = AC_PosControl.POSCONTROL_ACCEL_XY_MAX;

accel_target                        = AC_PosControl.accel_target;
accel_target_filter                 = AC_PosControl.accel_target_filter;
pos_target                          = AC_PosControl.pos_target;
pos_error                           = AC_PosControl.pos_error;
flags_vehicle_horiz_vel_override    = AC_PosControl.flags_vehicle_horiz_vel_override;
flags_reset_accel_to_lean_xy        = AC_PosControl.flags_reset_accel_to_lean_xy;
vehicle_horiz_vel                   = AC_PosControl.vehicle_horiz_vel;
vel_error                           = AC_PosControl.vel_error;
vel_desired                         = AC_PosControl.vel_desired;
vel_target                          = AC_PosControl.vel_target;
accel_desired                       = AC_PosControl.accel_desired;
limit_accel_xy                      = AC_PosControl.limit_accel_xy;
limit_throttle_upper                = AP_Motors.limit_throttle_upper; 

kP=p_pos_xy;
    % avoid divide by zero
    if (kP <= 0.0)
        vel_target(1) = 0.0;
        vel_target(2)= 0.0;
    else
        % calculate distance error
        pos_error(1) = pos_target(1) - curr_pos(1);
        pos_error(2) = pos_target(2) - curr_pos(2);
        
        % Constrain _pos_error and target position
        % Constrain the maximum length of _vel_target to the maximum position correction velocity
        % TODO: replace the leash length with a user definable maximum position correction
        [bool,pos_error(1),pos_error(2)]= limit_vector_length(pos_error(1), pos_error(2), leash);
        if (bool)
            pos_target(1) = curr_pos(1) + pos_error(1);
            pos_target(2) = curr_pos(2) + pos_error(2);
        end
        
        vel_target = sqrt_controller_pos(pos_error, kP, accel_cms);
        
    end
    % add velocity feed-forward
    vel_target(1) =vel_target(1) + vel_desired(1);
    vel_target(2) =vel_target(2) + vel_desired(2);

    % the following section converts desired velocities in lat/lon directions to accelerations in lat/lon frame

    % check if vehicle velocity is being overridden
    if (flags_vehicle_horiz_vel_override)
        flags_vehicle_horiz_vel_override = false;
    else
        vehicle_horiz_vel(1) = curr_vel(1);
        vehicle_horiz_vel(2) = curr_vel(2);
    end
     

    % calculate velocity error
    vel_error(1) =  vel_target(1) - vehicle_horiz_vel(1);
    vel_error(2) =  vel_target(2) - vehicle_horiz_vel(2);
    % TODO: constrain velocity error and velocity target

    % call pi controller
%     _pid_vel_xy.set_input(vel_error);
% 
%     % get p
%     vel_xy_p = _pid_vel_xy.get_p();
% 
%     % update i term if we have not hit the accel or throttle limits OR the i term will reduce
%     % TODO: move limit handling into the PI and PID controller
%     if (~limit_accel_xy && ~limit_throttle_upper)  
%         vel_xy_i = _pid_vel_xy.get_i();
%       else  
%         vel_xy_i = _pid_vel_xy.get_i_shrink();
%     end
% 
%     % get d
%     vel_xy_d = _pid_vel_xy.get_d();
% 
%     % acceleration to correct for velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
%     accel_target.x = (vel_xy_p.x + vel_xy_i.x + vel_xy_d.x) * ekfNavVelGainScaler;
%     accel_target.y = (vel_xy_p.y + vel_xy_i.y + vel_xy_d.y) * ekfNavVelGainScaler;
   accel_target(1:2) = pid_accel_xy_update_all(vel_target(1:2),vehicle_horiz_vel(1:2),(limit_accel_xy ||limit_throttle_upper));
    % reset accel to current desired acceleration
    if (flags_reset_accel_to_lean_xy)  
        accel_target_filter(1:2)=accel_target(1:2);
        flags_reset_accel_to_lean_xy = false;
    end

    % filter correction acceleration
%     accel_target_filter.set_cutoff_frequency(MIN(_accel_xy_filt_hz, 5.0f * ekfNavVelGainScaler));
%     accel_target_filter.apply(accel_target, dt);

          
    accel_target_filter(1:2)=accel_target_filter(1:2) + (accel_target(1:2) - accel_target_filter(1:2)) * get_filt_alpha(accel_xy_filt_hz);
    
    % pass the correction acceleration to the target acceleration output
    accel_target(1:2) = accel_target_filter(1:2);

    % Add feed forward into the target acceleration output
    accel_target(1:2)= accel_target(1:2)+accel_desired(1:2);
    % the following section converts desired accelerations provided in lat/lon frame to roll/pitch angles

    % limit acceleration using maximum lean angles
%      angle_max = min(get_althold_lean_angle_max(), get_lean_angle_max_cd());
     angle_max =get_althold_lean_angle_max();
     accel_max = min(GRAVITY_MSS * 100.0 * tan(angle_max * 0.01/HD), POSCONTROL_ACCEL_XY_MAX);
     [limit_accel_xy,accel_target(1),accel_target(2)] = limit_vector_length( accel_target(1),  accel_target(2), accel_max);
    % update angle targets that will be passed to stabilize controller
     [ roll_target, pitch_target]=accel_to_lean_angles(accel_target(1), accel_target(2));
     
 AC_PosControl.accel_target                            = accel_target;                      
 AC_PosControl.accel_target_filter                     = accel_target_filter;              
 AC_PosControl.pos_target                              = pos_target;                       
 AC_PosControl.pos_error                               = pos_error;                        
 AC_PosControl.flags_vehicle_horiz_vel_override        = flags_vehicle_horiz_vel_override; 
 AC_PosControl.flags_reset_accel_to_lean_xy            = flags_reset_accel_to_lean_xy;
 AC_PosControl.vehicle_horiz_vel                       = vehicle_horiz_vel;                
 AC_PosControl.vel_error                               = vel_error; 
 AC_PosControl.vel_desired                             = vel_desired;		 		               
 AC_PosControl.vel_target                              = vel_target;
 AC_PosControl.accel_desired                           = accel_desired;
 AC_PosControl.limit_accel_xy                          = limit_accel_xy;
 AC_PosControl.roll_target                             = roll_target;
 AC_PosControl.pitch_target                            = pitch_target;
 AP_Motors.limit_throttle_upper                        = limit_throttle_upper; 

end

