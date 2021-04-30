function run_z_controller()
%  run position control for Z axis
%  target altitude should be set with one of these functions: set_alt_target, set_target_to_stopping_point_z, init_takeoff
%  calculates desired rate in earth-frame z axis and passes to rate controller
%  vel_up_max, vel_down_max should have already been set before calling this method

global dt
global AC_PosControl
global AP_Motors
global Copter_Plane
global SINS

curr_alt                     = SINS.curr_alt;
curr_vel                     = SINS.curr_vel;
z_accel_meas                 = SINS.z_accel_meas;

p_pos_z                      = AC_PosControl.p_pos_z;
p_vel_z                      = AC_PosControl.p_vel_z;
speed_down_cms               = AC_PosControl.speed_down_cms;
speed_up_cms                 = AC_PosControl.speed_up_cms;
accel_z_cms                  = AC_PosControl.accel_z_cms;
leash_up_z                   = AC_PosControl.leash_up_z;
leash_down_z                 = AC_PosControl.leash_down_z;
vibe_comp_enabled            = AC_PosControl.vibe_comp_enabled;


flags_reset_rate_to_accel_z  = AC_PosControl.flags_reset_rate_to_accel_z;
flags_freeze_ff_z            = AC_PosControl.flags_freeze_ff_z;
flags_use_desvel_ff_z        = AC_PosControl.flags_use_desvel_ff_z;

pos_error                  = AC_PosControl.pos_error;
pos_target                 = AC_PosControl.pos_target;
vel_target                 = AC_PosControl.vel_target;
accel_target               = AC_PosControl.accel_target;

vel_desired                  = AC_PosControl.vel_desired;
vel_error_filter             = AC_PosControl.vel_error_filter;

vel_error                    = AC_PosControl.vel_error;
vel_last                     = AC_PosControl.vel_last;
accel_desired                = AC_PosControl.accel_desired;
vel_z_control_ratio          = AC_PosControl.vel_z_control_ratio;

POSCONTROL_THROTTLE_CUTOFF_FREQ    = AC_PosControl.POSCONTROL_THROTTLE_CUTOFF_FREQ;
POSCONTROL_VEL_ERROR_CUTOFF_FREQ   = AC_PosControl.POSCONTROL_VEL_ERROR_CUTOFF_FREQ;

limit_throttle_lower            = AP_Motors.limit_throttle_lower;
limit_throttle_upper            = AP_Motors.limit_throttle_upper;
throttle_hover                  = AP_Motors.throttle_hover;

take_off_land                   = Copter_Plane.take_off_land;
thr_out_min                     = Copter_Plane.thr_out_min;

% clear position limit flags
limit_pos_up = false;
limit_pos_down = false;

% calculate altitude error
pos_error(3) = pos_target(3) - curr_alt;

% do not let target altitude get too far from current altitude
if (pos_error(3) > leash_up_z)
    pos_target(3) = curr_alt + leash_up_z;
    pos_error(3) = leash_up_z;
    limit_pos_up = true;
end
if (pos_error(3) < -leash_down_z)
    pos_target(3) = curr_alt - leash_down_z;
    pos_error(3) = -leash_down_z;
    limit_pos_down = true;
end

% calculate _vel_target.z using from _pos_error.z using sqrt controller
vel_target(3) = sqrt_controller(pos_error(3), p_pos_z, accel_z_cms, dt);

% check speed limits
% To-Do: check these speed limits here or in the pos->rate controller
limit_vel_up = false;
limit_vel_down = false;
if (vel_target(3) < speed_down_cms)
    vel_target(3) = speed_down_cms;
    limit_vel_down = true;
end

if (vel_target(3) > speed_up_cms)
    vel_target(3) = speed_up_cms;
    limit_vel_up = true;
end

if(take_off_land)
    vel_target(3) =vel_desired(3);
else
    vel_target(3) =vel_target(3)+vel_desired(3);
end
% the following section calculates acceleration required to achieve the velocity target


% TODO: remove velocity derivative calculation
% reset last velocity target to current target
if (flags_reset_rate_to_accel_z)
    vel_last(3) = vel_target(3);
end

accel_desired(3) = 0.0;
% store this iteration's velocities for the next iteration
vel_last(3) = vel_target(3);

% reset velocity error and filter if this controller has just been engaged
if (flags_reset_rate_to_accel_z)
    % Reset Filter
    vel_error(3) = 0;
    vel_error_filter(3)=0;
    flags_reset_rate_to_accel_z = false;
else
    % calculate rate error and filter with cut off frequency of 2 Hz
    %         vel_error(3) = vel_error_filter(3).apply(vel_target(3) - curr_vel(3), dt);
    vel_error_in=vel_target(3) - curr_vel(3);
    vel_error_filter(3)=vel_error_filter(3) + (vel_error_in - vel_error_filter(3)) * get_filt_alpha(POSCONTROL_VEL_ERROR_CUTOFF_FREQ);
    vel_error(3)=vel_error_filter(3);
end

accel_target(3) = p_vel_z*vel_error(3);

accel_target(3) =accel_target(3)+accel_desired(3);


if (vibe_comp_enabled)
    flags_freeze_ff_z = true;
    accel_desired(3) = 0.0;
    [thr_out,AC_PosControl.pid_accel_z] = AC_PID_update_error(accel_target(3), (limit_throttle_lower || limit_throttle_upper),AC_PosControl.pid_accel_z);
    thr_out                 = thr_out*0.001;
else
    [thr_out,AC_PosControl.pid_accel_z] = AC_PID_update_all(accel_target(3), z_accel_meas, (limit_throttle_lower || limit_throttle_upper),AC_PosControl.pid_accel_z) ;
     thr_out =(AC_PosControl.pid_accel_z.kff*accel_target(3)+thr_out)*0.001;
end
thr_out =thr_out+ throttle_hover;
thr_out=constrain_value(thr_out,thr_out_min,1);

% send throttle to attitude controller with angle boost
set_throttle_out(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ);

% _speed_down_cms is checked to be non-zero when set
error_ratio = vel_error(3)/speed_down_cms;

vel_z_control_ratio = vel_z_control_ratio +dt*0.1*(0.5-error_ratio);
vel_z_control_ratio = constrain_value (vel_z_control_ratio, 0.0, 2.0);

AC_PosControl.flags_reset_rate_to_accel_z  = flags_reset_rate_to_accel_z;
AC_PosControl.flags_freeze_ff_z            = flags_freeze_ff_z;
AC_PosControl.flags_use_desvel_ff_z        = flags_use_desvel_ff_z;
AC_PosControl.pos_error                    = pos_error;
AC_PosControl.pos_target                   = pos_target;
AC_PosControl.vel_target                   = vel_target;
AC_PosControl.accel_target                 = accel_target;

AC_PosControl.vel_desired                  = vel_desired;
AC_PosControl.vel_error                    = vel_error;
AC_PosControl.vel_error_filter             = vel_error_filter;
AC_PosControl.vel_last                     = vel_last;
AC_PosControl.accel_desired                = accel_desired;
AC_PosControl.vel_z_control_ratio          = vel_z_control_ratio;

AC_PosControl.limit_pos_up                 = limit_pos_up;
AC_PosControl.limit_pos_down               = limit_pos_down;
AC_PosControl.limit_vel_up                 = limit_vel_up;
AC_PosControl.limit_vel_down               = limit_vel_down;

AP_Motors.limit_throttle_lower            = limit_throttle_lower;
AP_Motors.limit_throttle_upper            = limit_throttle_upper;
AP_Motors.throttle_hover                  = throttle_hover;

Copter_Plane.take_off_land                     = take_off_land;
end

