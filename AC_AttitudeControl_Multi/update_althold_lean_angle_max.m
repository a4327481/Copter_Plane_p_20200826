function update_althold_lean_angle_max( throttle_in )
% Update Alt_Hold angle maximum

global dt
global AP_Motors

throttle_thrust_max                                = AP_Motors.throttle_thrust_max;
althold_lean_angle_max                             = AP_Motors.althold_lean_angle_max;
angle_limit_tc                                     = AP_Motors.angle_limit_tc;
AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX       = AP_Motors.AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX;

    % calc maximum tilt angle based on throttle
    thr_max = throttle_thrust_max;  
    
    % divide by zero check
    if (thr_max==0)
        althold_lean_angle_max = 0.0;
        return
    end
    
    althold_lean_angle_maxi = acos(constrain_value(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0, 1.0));
    althold_lean_angle_max = althold_lean_angle_max + (dt / (dt + angle_limit_tc)) * (althold_lean_angle_maxi - althold_lean_angle_max);
        % divide by zero check
		
AP_Motors.throttle_thrust_max                                = throttle_thrust_max;
AP_Motors.althold_lean_angle_max                             = althold_lean_angle_max;
AP_Motors.angle_limit_tc                                     = angle_limit_tc;
AP_Motors.AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX       = AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX;		

end

