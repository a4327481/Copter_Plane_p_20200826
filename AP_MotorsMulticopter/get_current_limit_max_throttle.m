function   throttle_thrust_max=get_current_limit_max_throttle()
global batt_current_max
global armed
global throttle_limit
global batt_current
global batt_resistance
global throttle_hover
global batt_voltage
global batt_voltage_min
global batt_current_time_constant
global dt
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
end


