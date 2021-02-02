function update_lift_max_from_batt_voltage()
    % sanity check battery_voltage_min is not too small
    % if disabled or misconfigured exit immediately
 global batt_voltage_resting_estimate 
 global batt_voltage_max
 global batt_voltage_min
 global batt_voltage_filt
 global lift_max
 global loop_rate
 global thrust_curve_expo
 global dt
    if ((batt_voltage_max <= 0) || (batt_voltage_min >= batt_voltage_max) || (batt_voltage_resting_estimate < 0.25 * batt_voltage_min))  
        batt_voltage_filt=1;
        lift_max = 1.0;
        return;
    end

    batt_voltage_min = max(batt_voltage_min, batt_voltage_max * 0.6);

    % contrain resting voltage estimate (resting voltage is actual voltage with sag removed based on current draw and resistance)
    batt_voltage_resting_estimate = constrain_value (batt_voltage_resting_estimate, batt_voltage_min, batt_voltage_max);

    % filter at 0.5 Hz
%       batt_voltage_filt = batt_voltage_filt.apply(batt_voltage_resting_estimate / batt_voltage_max, 1.0 / loop_rate);
        rc=1/(2*pi*0.5);
        alpha = constrain_value(dt/(dt+rc), 0.0, 1.0);
        batt_voltage_filt= batt_voltage_filt+(batt_voltage_resting_estimate / batt_voltage_max - batt_voltage_filt) * alpha;
      
    % calculate lift max
      thrust_curve_expot = constrain_value (thrust_curve_expo, -1.0, 1.0);
    lift_max = batt_voltage_filt * (1 - thrust_curve_expot) + thrust_curve_expot * batt_voltage_filt * batt_voltage_filt;
 

end

