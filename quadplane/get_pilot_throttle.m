function get_pilot_throttle()
 global throttle_in
 global throttle_hover
 global throttle_expo
    % get scaled throttle input

    % normalize to [0,1]

    if (is_positive(throttle_expo))  
        % get hover throttle level [0,1]
          thr_mid = throttle_hover;
          thrust_curve_expo = constrain_value (throttle_expo, 0.0, 1.0);

        % this puts mid stick at hover throttle
         throttle_in=throttle_curve(thr_mid, thrust_curve_expo, throttle_in);        
    end
 
end

