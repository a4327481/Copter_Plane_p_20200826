function thrust_o=apply_thrust_curve_and_volt_scaling( thrust) 

global AP_Motors


 thrust_curve_expo               = AP_Motors.thrust_curve_expo;
 batt_voltage_filt               = AP_Motors.batt_voltage_filt;
 lift_max                        = AP_Motors.lift_max;
 
 
    % apply thrust curve - domain 0.0 to 1.0, range 0.0 to 1.0
      thrust_curve_expo = constrain_value (thrust_curve_expo, -1.0, 1.0);
    if (abs(thrust_curve_expo) < 0.001)  
        % zero expo means linear, avoid  ing point exception for small values
        thrust_o= thrust;
        return;
    end
    if ((batt_voltage_filt~=0))  
        throttle_ratio = ((thrust_curve_expo - 1.0) + sqrt((1.0 - thrust_curve_expo) * (1.0 - thrust_curve_expo) + 4.0 * thrust_curve_expo * lift_max * thrust)) / (2.0 * thrust_curve_expo * batt_voltage_filt);  
    else  
        throttle_ratio = ((thrust_curve_expo - 1.0) + sqrt((1.0 - thrust_curve_expo) * (1.0 - thrust_curve_expo) + 4.0 * thrust_curve_expo * lift_max * thrust)) / (2.0 * thrust_curve_expo);
    end

    thrust_o= constrain_value (throttle_ratio, 0.0, 1.0);
 
 AP_Motors.thrust_curve_expo               = thrust_curve_expo;
 AP_Motors.batt_voltage_filt               = batt_voltage_filt;
 AP_Motors.lift_max                        = lift_max;

end

