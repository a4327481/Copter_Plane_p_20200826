function  set_throttle_out(  throttle_ini,   apply_angle_boost,   filter_cutoff)

global AP_Motors

 throttle_in                        = AP_Motors.throttle_in;
 angle_boost                        = AP_Motors.angle_boost;
 throttle_avg_max                   = AP_Motors.throttle_avg_max;
 throttle_cutoff_frequency          = AP_Motors.throttle_cutoff_frequency;
 
    throttle_in = throttle_ini;
    update_althold_lean_angle_max(throttle_ini);
    throttle_cutoff_frequency=filter_cutoff;
    if (apply_angle_boost)  
        % Apply angle boost
        throttle_ini = get_throttle_boosted(throttle_ini);
      else  
        % Clear angle_boost for logging purposes
        angle_boost = 0.0;
    end
%     set_throttle(throttle_ini);
    throttle_avg_max=constrain_value(get_throttle_avg_max(max(throttle_ini, throttle_in)),0,1);
 
 AP_Motors.throttle_in                        = throttle_ini;
 AP_Motors.angle_boost                        = angle_boost;
 AP_Motors.throttle_avg_max                   = throttle_avg_max;
 AP_Motors.throttle_cutoff_frequency          = throttle_cutoff_frequency;
 
 
end

