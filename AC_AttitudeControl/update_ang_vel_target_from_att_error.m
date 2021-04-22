function rate_target_ang_velo = update_ang_vel_target_from_att_error(attitude_error_rot_vec_radin)
    global dt
	global AC_Attitude
	
    use_sqrt_controller                                  = AC_Attitude.use_sqrt_controller;  
    accel_roll_max                                       = AC_Attitude.accel_roll_max;
    accel_pitch_max                                      = AC_Attitude.accel_pitch_max;
    accel_yaw_max                                        = AC_Attitude.accel_yaw_max;
    p_angle_roll                                         = AC_Attitude.p_angle_roll;
    p_angle_pitch                                        = AC_Attitude.p_angle_pitch;
    p_angle_yaw                                          = AC_Attitude.p_angle_yaw;
    AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS            = AC_Attitude.AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS;
    AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS            = AC_Attitude.AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS;
    AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS             = AC_Attitude.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS;
    AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS             = AC_Attitude.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS;
	
    % Compute the roll angular velocity demand from the roll angle error
    attitude_error_rot_vec_rad.x=attitude_error_rot_vec_radin(1);
    attitude_error_rot_vec_rad.y=attitude_error_rot_vec_radin(2);
    attitude_error_rot_vec_rad.z=attitude_error_rot_vec_radin(3);
    if (use_sqrt_controller) 
        rate_target_ang_vel.x = sqrt_controller(attitude_error_rot_vec_rad.x, p_angle_roll, constrain_value(radians(accel_roll_max * 0.01) / 2.0, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS), dt);
     else 
        rate_target_ang_vel.x = p_angle_roll * attitude_error_rot_vec_rad.x;
    end
     %todo: Add Angular Velocity Limit

     %Compute the pitch angular velocity demand from the pitch angle error
    if (use_sqrt_controller) 
        rate_target_ang_vel.y = sqrt_controller(attitude_error_rot_vec_rad.y, p_angle_pitch, constrain_value(radians(accel_pitch_max*0.01)/ 2.0, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS), dt);
     else 
        rate_target_ang_vel.y = p_angle_pitch * attitude_error_rot_vec_rad.y;
    end
%      todo: Add Angular Velocity Limit

%      Compute the yaw angular velocity demand from the yaw angle error
    if (use_sqrt_controller) 
        rate_target_ang_vel.z = sqrt_controller(attitude_error_rot_vec_rad.z, p_angle_yaw, constrain_value(radians(accel_yaw_max*0.01) / 2.0, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS), dt);
     else 
        rate_target_ang_vel.z = p_angle_yaw * attitude_error_rot_vec_rad.z;
    end
%      todo: Add Angular Velocity Limit
rate_target_ang_velo=[rate_target_ang_vel.x rate_target_ang_vel.y rate_target_ang_vel.z];

    AC_Attitude.use_sqrt_controller                                  = use_sqrt_controller;  
    AC_Attitude.accel_roll_max                                       = accel_roll_max;
    AC_Attitude.accel_pitch_max                                      = accel_pitch_max;
    AC_Attitude.accel_yaw_max                                        = accel_yaw_max;
    AC_Attitude.p_angle_roll                                         = p_angle_roll;
    AC_Attitude.p_angle_pitch                                        = p_angle_pitch;
    AC_Attitude.p_angle_yaw                                          = p_angle_yaw;
    AC_Attitude.AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS            = AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS;
    AC_Attitude.AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS            = AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS;
    AC_Attitude.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS             = AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS;
    AC_Attitude.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS             = AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS;
end

