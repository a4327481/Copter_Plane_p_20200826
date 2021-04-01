function   throttle_thrust_max=get_current_limit_max_throttle()

global Copter_Plane
global dt
global AP_Motors

armed                                = Copter_Plane.armed;
throttle_hover                       = AP_Motors.throttle_hover;
throttle_limit                       = AP_Motors.throttle_limit;
batt_current_max                     = AP_Motors.batt_current_max;
batt_current                         = AP_Motors.batt_current;
batt_resistance                      = AP_Motors.batt_resistance;
batt_voltage                         = AP_Motors.batt_voltage;
batt_voltage_min                     = AP_Motors.batt_voltage_min;
batt_current_time_constant           = AP_Motors.batt_current_time_constant;



if (batt_current_max <= 0 ||~armed || ~batt_current)
    throttle_limit = 1.0;
    throttle_thrust_max=1;
    return ;
end

if (is_zero(batt_resistance))
    throttle_limit = 1.0;
    throttle_thrust_max=1;
    return ;
end


% calculate the maximum current to prevent voltage sag below _batt_voltage_min
batt_current_max = min(batt_current_max, batt_current + (batt_voltage - batt_voltage_min) / batt_resistance);

batt_current_ratio = batt_current / batt_current_max;

loop_interval = 1.0 / (1/dt);
throttle_limit= throttle_limit+ (loop_interval / (loop_interval + batt_current_time_constant)) * (1.0 - batt_current_ratio);

% throttle limit drops to 20% between hover and full throttle
throttle_limit = constrain_value (throttle_limit, 0.2, 1.0);

% limit max throttle
throttle_thrust_max= throttle_hover + ((1.0 - throttle_hover) * throttle_limit);

AP_Motors.throttle_hover                       = throttle_hover;
AP_Motors.throttle_limit                       = throttle_limit;
AP_Motors.batt_current_max                     = batt_current_max;
AP_Motors.batt_current                         = batt_current;
AP_Motors.batt_resistance                      = batt_resistance;
AP_Motors.batt_voltage                         = batt_voltage;
AP_Motors.batt_voltage_min                     = batt_voltage_min;
AP_Motors.batt_current_time_constant           = batt_current_time_constant;

end


