function get_pilot_throttle()

global AP_Motors
 throttle_in                 = AP_Motors.throttle_in;
 throttle_hover              = AP_Motors.throttle_hover;
 throttle_expo               = AP_Motors.throttle_expo;
    % get scaled throttle input

    % normalize to [0,1]

    if ((throttle_expo)>0)  
        % get hover throttle level [0,1]
          thr_mid = throttle_hover;
          thrust_curve_expo = constrain_value (throttle_expo, 0.0, 1.0);

        % this puts mid stick at hover throttle
         throttle_in=throttle_curve(thr_mid, thrust_curve_expo, throttle_in);        
    end
    
 AP_Motors.throttle_in                  = throttle_in;
 AP_Motors.throttle_hover               = throttle_hover ;
 AP_Motors.throttle_expo                = throttle_expo;  
end

