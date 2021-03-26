function update_throttle_mix()
global armed
global HD
global z_accel_meas
global vel_z_control_ratio
global vel_desired
global attitude_target_euler_angle
global thrust_error_angle

global AP_Motors

throttle_rpy_mix_desired                  = AP_Motors.throttle_rpy_mix_desired;
thr_mix_min                               = AP_Motors.thr_mix_min;
LAND_CHECK_LARGE_ANGLE_CD                 = AP_Motors.LAND_CHECK_LARGE_ANGLE_CD;
LAND_CHECK_ANGLE_ERROR_DEG                = AP_Motors.LAND_CHECK_ANGLE_ERROR_DEG;
LAND_CHECK_ACCEL_MOVING                   = AP_Motors.LAND_CHECK_ACCEL_MOVING;
land_accel_ef_filter                      = AP_Motors.land_accel_ef_filter;
 
     % if disarmed or landed prioritise throttle
    if (~armed) 
        throttle_rpy_mix_desired = thr_mix_min;
        return;
    end
    
%     if (flightmode->has_manual_throttle()) 
%         % manual throttle
%         if (channel_throttle->get_control_in() <= 0 || air_mode == AirMode::AIRMODE_DISABLED) 
%             attitude_control->set_throttle_mix_min();
%          else 
%             attitude_control->set_throttle_mix_man();
%         
%      else 
        % autopilot controlled throttle

        % check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        angle_target = attitude_target_euler_angle*HD;
        large_angle_request = (norm([angle_target(1), angle_target(2)],2) > LAND_CHECK_LARGE_ANGLE_CD);

        % check for large external disturbance - angle error over 30 degrees
        angle_error = thrust_error_angle*HD;
        large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);

        % check for large acceleration - falling or high turbulence
        
        land_accel_ef_filter=land_accel_ef_filter + (z_accel_meas - land_accel_ef_filter) * get_filt_alpha(1);
          accel_moving = (norm(land_accel_ef_filter,2) > LAND_CHECK_ACCEL_MOVING);

        % check for requested decent
          descent_not_demanded = vel_desired(3) >= 0.0;

        % check if landing
%           landing = flightmode->is_landing();
          landing = 0;
        if ((large_angle_request && ~landing) || large_angle_error || accel_moving || descent_not_demanded) 
            set_throttle_mix_max(constrain_value(vel_z_control_ratio, 0.0, 1.0));
         else 
            throttle_rpy_mix_desired = thr_mix_min;
        end

AP_Motors.throttle_rpy_mix_desired                  = throttle_rpy_mix_desired;
AP_Motors.thr_mix_min                               = thr_mix_min;
AP_Motors.LAND_CHECK_LARGE_ANGLE_CD                 = LAND_CHECK_LARGE_ANGLE_CD;
AP_Motors.LAND_CHECK_ANGLE_ERROR_DEG                = LAND_CHECK_ANGLE_ERROR_DEG;
AP_Motors.LAND_CHECK_ACCEL_MOVING                   = LAND_CHECK_ACCEL_MOVING;
AP_Motors.land_accel_ef_filter                      = land_accel_ef_filter;   
 
end

