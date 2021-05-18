function init_vel_controller_xyz()
% init_vel_controller_xyz - initialise the velocity controller - should be called once before the caller attempts to use the controller
global AC_PosControl
global SINS
global HD

AC_PosControl.roll_target                                    =  SINS.roll*HD*100;
AC_PosControl.pitch_target                                   =  SINS.pitch*HD*100;
% set roll, pitch lean angle targets to current attitude
AC_PosControl.pid_vel_xy.flags_reset_filter                 = true;
AC_PosControl.pid_accel_z.flags_reset_filter                = true;

AC_PosControl.accel_target(1:2)                             = lean_angles_to_accel();
AC_PosControl.pid_vel_xy.integrator                         = AC_PosControl.accel_target(1:2);
% flag reset required in rate to accel step
AC_PosControl.flags_reset_desired_vel_to_pos                 = true;
AC_PosControl.flags_reset_accel_to_lean_xy                   = true;

% set target position
AC_PosControl.pos_target(1:2)                                = SINS.curr_pos(1:2);
AC_PosControl.pos_target(3)                                  = SINS.curr_alt;
% move current vehicle velocity into feed forward velocity
AC_PosControl.vel_target                                     = SINS.curr_vel;
AC_PosControl.vel_desired                                    = [0 0 0];
% set vehicle acceleration to zero
AC_PosControl.accel_desired(1:2)                             = [0 0];
% initialise ekf reset handlers
end

