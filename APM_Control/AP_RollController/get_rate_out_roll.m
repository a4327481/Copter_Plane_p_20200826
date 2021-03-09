function rate_out=get_rate_out_roll(  desired_rate,   scaler,   disable_integrator)
global dt
global gyro_x
global HD
global aspeed
global airspeed_min
global EAS2TAS
global AP_rate_roll

gains_P              = AP_rate_roll.gains_P;
gains_I              = AP_rate_roll.gains_I;
gains_D              = AP_rate_roll.gains_D;
gains_FF             = AP_rate_roll.gains_FF;
gains_tau            = AP_rate_roll.gains_tau;
gains_imax           = AP_rate_roll.gains_imax;
slew_rate_max        = AP_rate_roll.slew_rate_max;
slew_rate_tau        = AP_rate_roll.slew_rate_tau;

last_out             = AP_rate_roll.last_out;
pid_info_target      = AP_rate_roll.pid_info_target;
pid_info_actual      = AP_rate_roll.pid_info_actual;
pid_info_error       = AP_rate_roll.pid_info_error;
pid_info_P           = AP_rate_roll.pid_info_P;
pid_info_I           = AP_rate_roll.pid_info_I;
pid_info_D           = AP_rate_roll.pid_info_D;
pid_info_FF          = AP_rate_roll.pid_info_FF;
last_pid_info_D      = AP_rate_roll.last_pid_info_D;

slew_filterg         = AP_rate_roll.slew_filterg;
slew_rate_amplitude  = AP_rate_roll.slew_rate_amplitude;
D_gain_modifier      = AP_rate_roll.D_gain_modifier;
pid_info_Dmod        = AP_rate_roll.pid_info_Dmod;
	% Calculate equivalent gains so that values for K_P and K_I can be taken across from the old PID law
    % No conversion is required for K_D
	  ki_rate = gains.I * gains_tau;
      eas2tas =EAS2TAS;
	  kp_ff = max((gains_P - gains_I * gains_tau) * gains_tau  - gains_D , 0) / eas2tas;
      k_ff = gains_FF / eas2tas;
	  delta_time = dt;
    % Get body rate vector (radians/sec)
	  omega_x = gyro_x;
	
	% Calculate the roll rate error (deg/sec) and apply gain scaler
      achieved_rate = omega_x*HD;
      pid_info_error = desired_rate - achieved_rate;
      rate_error = pid_info_error * scaler;
      pid_info_target = desired_rate;
      pid_info_actual = achieved_rate;
	
	% Get an airspeed estimate - default to zero if none available
	   
    

	% Multiply roll rate error by _ki_rate, apply scaler and integrate
	% Scaler is applied before integrator so that integrator state relates directly to aileron deflection
	% This means aileron trim offset doesn't change as the value of scaler changes with airspeed
	% Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
    if (~disable_integrator && ki_rate > 0)
        %only integrate if gain and time step are positive and airspeed above min value.
        if (dt > 0 && aspeed >  (airspeed_min))
            integrator_delta = rate_error * ki_rate * delta_time * scaler;
            % prevent the integrator from increasing if surface defln demand is above the upper limit
            if (last_out < -45)
                integrator_delta = max(integrator_delta , 0);
            elseif (last_out > 45)
                % prevent the integrator from decreasing if surface defln demand  is below the lower limit
                integrator_delta = min(integrator_delta, 0);
            end
            pid_info_I =pid_info_I+ integrator_delta;
        end
    else
        pid_info_I = 0;
    end
	
    % Scale the integration limit
      intLimScaled = gains_imax * 0.01;

    % Constrain the integrator state
    pid_info_I = constrain_value (pid_info_I, -intLimScaled, intLimScaled);
	
	% Calculate the demanded control surface deflection
	% Note the scaler is applied again. We want a 1/speed scaler applied to the feed-forward
	% path, but want a 1/speed^2 scaler applied to the rate error path. 
	% This is because acceleration scales with speed^2, but rate scales with speed.
    pid_info_D = rate_error * gains_D * scaler;
    pid_info_P = desired_rate * kp_ff * scaler;
    pid_info_FF = desired_rate * k_ff * scaler;

    if (dt > 0 && slew_rate_max > 0) 
        % Calculate the slew rate amplitude produced by the unmodified D term

        % calculate a low pass filtered slew rate
%           Dterm_slew_rate = _slew_rate_filter.apply((fabsf(_pid_info.D - _last_pid_info_D)/ delta_time), delta_time);
         slew_filter_in=abs(pid_info_D - last_pid_info_D)/ delta_time;
         slew_filterg=slew_filterg + (slew_filter_in - slew_filterg) * get_filt_alpha(10);
         Dterm_slew_rate=slew_filterg;
        % rectify and apply a decaying envelope filter
         alpha = 1.0 - constrain_value (delta_time/slew_rate_tau, 0.0 , 1.0);
         slew_rate_amplitude = max(abs(Dterm_slew_rate), alpha * slew_rate_amplitude);
         slew_rate_amplitude = min(slew_rate_amplitude, 10.0*slew_rate_max);

        % Calculate and apply the D gain adjustment 
        D_gain_modifier = slew_rate_max / max(slew_rate_amplitude, slew_rate_max);
        pid_info_Dmod = D_gain_modifier;
        pid_info_D =pid_info_D*D_gain_modifier;
    end

    last_pid_info_D = pid_info_D;

    last_out = pid_info_D + pid_info_FF + pid_info_P;

%     if (autotune.running && aspeed > aparm.airspeed_min) 
%         % let autotune have a go at the values 
%         % Note that we don't pass the integrator component so we get
%         % a better idea of how much the base PD controller
%         % contributed
%         autotune.update(desired_rate, achieved_rate, _last_out);
%     end
    

	last_out =last_out+ pid_info_I;
	
	% Convert to centi-degrees and constrain
	rate_out= constrain_value (last_out * 100, -4500, 4500);
    
AP_rate_roll.last_out                    = last_out;
AP_rate_roll.pid_info_target             = pid_info_target;
AP_rate_roll.pid_info_actual             = pid_info_actual;
AP_rate_roll.pid_info_error              = pid_info_error;
AP_rate_roll.pid_info_P                  = pid_info_P;
AP_rate_roll.pid_info_I                  = pid_info_I;
AP_rate_roll.pid_info_D                  = pid_info_D;
AP_rate_roll.pid_info_FF                 = pid_info_FF;
AP_rate_roll.last_pid_info_D             = last_pid_info_D;

AP_rate_roll.slew_filterg                = slew_filterg;
AP_rate_roll.slew_rate_amplitude         = slew_rate_amplitude;
AP_rate_roll.D_gain_modifier             = D_gain_modifier;
AP_rate_roll.pid_info_Dmod               = pid_info_Dmod;
end

