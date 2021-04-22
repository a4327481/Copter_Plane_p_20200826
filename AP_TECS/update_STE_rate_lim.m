function  update_STE_rate_lim( )

 global GRAVITY_MSS
 global AP_TECS
 
 maxClimbRate            = AP_TECS.maxClimbRate;
 minSinkRate             = AP_TECS.minSinkRate;
 
    % Calculate Specific Total Energy Rate Limits
    % This is a trivial calculation at the moment but will get bigger once we start adding altitude effects
    STEdot_max = maxClimbRate * GRAVITY_MSS;
    STEdot_min = - minSinkRate * GRAVITY_MSS;
	
 AP_TECS.STEdot_max              = STEdot_max;     
 AP_TECS.STEdot_min              = STEdot_min;
 
end

