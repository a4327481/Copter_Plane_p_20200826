function servo_out=get_servo_out_roll(  angle_err,   scaler,   disable_integrator)
%  global desired_rate_roll
 global AP_rate_roll
 
 gains_tau   =AP_rate_roll.gains_tau;
 gains_rmax = AP_rate_roll.gains_rmax;
 
%  Function returns an equivalent aileron deflection in centi-degrees in the range from -4500 to 4500
%  A positive demand is up
%  Inputs are: 
%  1) demanded bank angle in centi-degrees
%  2) control gain scaler = scaling_speed / aspeed
%  3) boolean which is true when stabilise mode is active
%  4) minimum FBW airspeed (metres/sec)
  
 
    if (gains_tau < 0.1)  
        gains_tau=0.1;
    end
	
	% Calculate the desired roll rate (deg/sec) from the angle error
	desired_rate = angle_err * 0.01 / gains_tau;
%     desired_rate_roll=desired_rate;
        % Limit the demanded roll rate
        if (gains_rmax && desired_rate < -gains_rmax)
            desired_rate = - gains_rmax;
        elseif (gains_rmax && desired_rate > gains_rmax)
            desired_rate = gains_rmax;
        end
    servo_out=get_rate_out_roll(desired_rate, scaler, disable_integrator);
 
AP_rate_roll.gains_tau   = gains_tau;
AP_rate_roll.gains_rmax  = gains_rmax ;
end

