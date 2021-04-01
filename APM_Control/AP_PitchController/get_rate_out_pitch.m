function  rate_out=get_rate_out_pitch(  desired_rate,   scaler,   disable_integrator)
global dt
global gyro_y
global HD   
global aspeed
global EAS2TAS
global roll
global pitch
global AP_rate_pitch
global Plane
global Copter_Plane

 gains_P                  =AP_rate_pitch.gains_P;
 gains_I                  =AP_rate_pitch.gains_I;
 gains_D                  =AP_rate_pitch.gains_D;
 gains_FF                 =AP_rate_pitch.gains_FF;
 gains_tau                =AP_rate_pitch.gains_tau;
 gains_imax               =AP_rate_pitch.gains_imax;
 slew_rate_max            =AP_rate_pitch.slew_rate_max;
 slew_rate_tau            =AP_rate_pitch.slew_rate_tau;
  
 last_out                 =AP_rate_pitch.last_out;
 pid_info_target          =AP_rate_pitch.pid_info_target;
 pid_info_actual          =AP_rate_pitch.pid_info_actual;
 pid_info_error           =AP_rate_pitch.pid_info_error;
 pid_info_P               =AP_rate_pitch.pid_info_P;
 pid_info_I               =AP_rate_pitch.pid_info_I;
 pid_info_D               =AP_rate_pitch.pid_info_D;
 pid_info_FF              =AP_rate_pitch.pid_info_FF;
 last_pid_info_D          =AP_rate_pitch.last_pid_info_D ;

 slew_filterg             =AP_rate_pitch.slew_filterg;       
 slew_rate_amplitude      =AP_rate_pitch.slew_rate_amplitude;
 D_gain_modifier          =AP_rate_pitch.D_gain_modifier;
 pid_info_Dmod            =AP_rate_pitch.pid_info_Dmod;
 airspeed_min             =Plane.airspeed_min; 
 roll_limit_cd	          =Plane.roll_limit_cd;
 disable_AP_rate_pitch_gains_D         = Copter_Plane.disable_AP_rate_pitch_gains_D;

 if(disable_AP_rate_pitch_gains_D)
    gains_D = 0;
 end
 
	  delta_time    =dt;	
	% Get body rate vector (radians/sec)
	% Calculate the pitch rate error (deg/sec) and scale
      achieved_rate = gyro_y*HD;
      pid_info_error = desired_rate - achieved_rate;
      rate_error =  pid_info_error * scaler;
      pid_info_target = desired_rate;
      pid_info_actual = achieved_rate;
	
	% Multiply pitch rate error by _ki_rate and integrate
	% Scaler is applied before integrator so that integrator state relates directly to elevator deflection
	% This means elevator trim offset doesn't change as the value of scaler changes with airspeed
	% Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
    if (~disable_integrator && gains_I > 0)
        k_I = gains_I;
        if (is_zero(gains_FF))
            
            %               if the user hasn't set a direct FF then assume they are
            %               not doing sophisticated tuning. Set a minimum I value of
            %               0.15 to ensure that the time constant for trimming in
            %               pitch is not too long. We have had a lot of user issues
            %               with very small I value leading to very slow pitch
            %               trimming, which causes a lot of problems for TECS. A
            %               value of 0.15 is still quite small, but a lot better
            %               than what many users are running.
            k_I = max(k_I, 0.15);
        end
        ki_rate = k_I * gains_tau;
        %only integrate if gain and time step are positive and airspeed above min value.
        if (dt > 0 && aspeed > 0.5*airspeed_min)
            integrator_delta = rate_error * ki_rate * delta_time * scaler;
            if (last_out < -45)
                % prevent the integrator from increasing if surface defln demand is above the upper limit
                integrator_delta = max(integrator_delta , 0);
            elseif (last_out > 45)
                % prevent the integrator from decreasing if surface defln demand  is below the lower limit
                integrator_delta = min(integrator_delta , 0);
            end
            pid_info_I =pid_info_I+integrator_delta;
        end
        
    else
        pid_info_I = 0;
        last_pid_info_D = rate_error * gains_D * scaler;
        slew_filterg = 0;
    end
    	
    % Scale the integration limit
    intLimScaled = gains_imax * 0.01;

    % Constrain the integrator state
    pid_info_I = constrain_value(pid_info_I, -intLimScaled, intLimScaled);

	% Calculate equivalent gains so that values for K_P and K_I can be taken across from the old PID law
    % No conversion is required for K_D
     eas2tas = EAS2TAS;
	 kp_ff = max((gains_P - gains_I * gains_tau) * gains_tau  - gains_D , 0) / eas2tas;
     k_ff = gains_FF / eas2tas;

	% Calculate the demanded control surface deflection
	% Note the scaler is applied again. We want a 1/speed scaler applied to the feed-forward
	% path, but want a 1/speed^2 scaler applied to the rate error path. 
	% This is because acceleration scales with speed^2, but rate scales with speed.
    pid_info_P = desired_rate * kp_ff * scaler;
    pid_info_FF = desired_rate * k_ff * scaler;
    pid_info_D = rate_error * gains_D * scaler;

    if (dt > 0 && slew_rate_max > 0) 
        % Calculate the slew rate amplitude produced by the unmodified D term

        % calculate a low pass filtered slew rate
%          Dterm_slew_rate = _slew_rate_filter.apply((fabsf(_pid_info.D - _last_pid_info_D)/ delta_time), delta_time);
        slew_filter_in=abs(pid_info_D - last_pid_info_D)/ delta_time;
        slew_filterg=slew_filterg + (slew_filter_in - slew_filterg) * get_filt_alpha(10);
        Dterm_slew_rate=slew_filterg;
        % rectify and apply a decaying envelope filter
        alpha = 1.0 - constrain_value(delta_time/slew_rate_tau, 0.0 , 1.0);
        slew_rate_amplitude = max(abs(Dterm_slew_rate), alpha * slew_rate_amplitude);
        slew_rate_amplitude = min(slew_rate_amplitude, 10.0*slew_rate_max);

        % Calculate and apply the D gain adjustment
        
        D_gain_modifier = slew_rate_max / max(slew_rate_amplitude, slew_rate_max);
        pid_info_Dmod = D_gain_modifier;
        pid_info_D=pid_info_D *D_gain_modifier;
    end

       last_pid_info_D = pid_info_D;
       last_out = pid_info_D + pid_info_FF + pid_info_P;

%     if (autotune.running && aspeed > aparm.airspeed_min) 
%         % let autotune have a go at the values 
%         % Note that we don't pass the integrator component so we get
%         % a better idea of how much the base PD controller
%         % contributed
%         autotune.update(desired_rate, achieved_rate, _last_out);
%         
%         % set down rate to rate up when auto-tuning
%         _max_rate_neg.set_and_save_ifchanged(gains.rmax);

	  last_out=last_out + pid_info_I;
 
%       when we are past the users defined roll limit for the
%       aircraft our priority should be to bring the aircraft back
%       within the roll limit. Using elevator for pitch control at
%       large roll angles is ineffective, and can be counter
%       productive as it induces earth-frame yaw which can reduce
%       the ability to roll. We linearly reduce elevator input when
%       beyond the configured roll limit, reducing to zero at 90
%       degrees
    
      roll_wrapped = abs(roll*HD*100);
    if (roll_wrapped > 9000) 
        roll_wrapped = 18000 - roll_wrapped;
    end
    if (roll_wrapped > roll_limit_cd + 500 && roll_limit_cd < 8500 && abs(pitch*HD*100) < 7000) 
        roll_prop = (roll_wrapped - (roll_limit_cd+500)) / (9000 - roll_limit_cd);
        last_out =last_out* (1 - roll_prop);
    end
    
	% Convert to centi-degrees and constrain
	 rate_out=constrain_value(last_out * 100, -4500, 4500);



AP_rate_pitch.last_out                    = last_out;
AP_rate_pitch.pid_info_target             = pid_info_target;
AP_rate_pitch.pid_info_actual             = pid_info_actual;
AP_rate_pitch.pid_info_error              = pid_info_error;
AP_rate_pitch.pid_info_P                  = pid_info_P;
AP_rate_pitch.pid_info_I                  = pid_info_I;
AP_rate_pitch.pid_info_D                  = pid_info_D;
AP_rate_pitch.pid_info_FF                 = pid_info_FF;
AP_rate_pitch.last_pid_info_D             = last_pid_info_D;

AP_rate_pitch.slew_filterg                = slew_filterg;
AP_rate_pitch.slew_rate_amplitude         = slew_rate_amplitude;
AP_rate_pitch.D_gain_modifier             = D_gain_modifier;
AP_rate_pitch.pid_info_Dmod               = pid_info_Dmod;
 
Plane.airspeed_min                        = airspeed_min;
Plane.roll_limit_cd	                      = roll_limit_cd;
Copter_Plane.disable_AP_rate_pitch_gains_D = disable_AP_rate_pitch_gains_D;
 
end

