function output=rate_target_to_motor_yawg( target_in,  measurement,  limit)
 global rate_yaw_pid
 global dt
 
 kp                      =rate_yaw_pid.kp;
 ki                      =rate_yaw_pid.ki;
 kd                      =rate_yaw_pid.kd;
 kff                     =rate_yaw_pid.kff;
 kimax                   =rate_yaw_pid.kimax;
 filt_T_hz               =rate_yaw_pid.filt_T_hz;
 filt_E_hz               =rate_yaw_pid.filt_E_hz;
 filt_D_hz               =rate_yaw_pid.filt_D_hz;
 slew_rate_max           =rate_yaw_pid.slew_rate_max;
 slew_rate_tau           =rate_yaw_pid.slew_rate_tau;
 
 flags_reset_filter      =rate_yaw_pid.flags_reset_filter;
 target                  =rate_yaw_pid.target;
 error                   =rate_yaw_pid.error;
 error_last              =rate_yaw_pid.error_last;
 integrator              =rate_yaw_pid.integrator;
 derivative              =rate_yaw_pid.derivative;
 slew_amplitude          =rate_yaw_pid.slew_amplitude;
 slew_filter             =rate_yaw_pid.slew_filterg;
 last_sample             =rate_yaw_pid.last_sample;
 Dmod                    =rate_yaw_pid.Dmod;
    % don't process inf or NaN

    % reset input filter to value received
    if (flags_reset_filter)  
        flags_reset_filter = false;
        target = target_in;
        error = target - measurement;
        derivative = 0.0;
        integrator = 0;
        slew_filter=0;
        last_sample=error * kp+derivative * kd;
      else  
        error_last = error;
        target=target + get_filt_alpha(filt_T_hz) * (target_in - target);
        error =error  + get_filt_alpha(filt_E_hz) * ((target - measurement) - error);
    end
        % calculate and filter derivative
        if (dt > 0.0)  
            derivative = (error - error_last) / dt;
            derivative=derivative + get_filt_alpha(filt_D_hz) * (derivative - derivative);
        end 
    % update I term
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (ki~=0 )  
        % Ensure that integrator can only be reduced if the output is saturated
        if (~limit || (((integrator>0) && (error<0)) || ((integrator<0) && (error>0))))                  
            integrator = integrator + (error * ki) * dt;
            integrator = constrain_value(integrator, -kimax, kimax);
        end      
    else
        integrator = 0.0;    
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     P_out = (error * kp);
     D_out = (derivative * kd);

    % calculate slew limit modifier for P+D
%     Dmodg = rate_yaw_pid_modifier(P_out + D_out, dt);
    sample=P_out + D_out;
%   apply filter to sample, returning multiplier between 0 and 1 to keep
%   output within slew rate
 
%    modifier(float sample, float dt)
 
    if (slew_rate_max <= 0)  
        Dmod= 1.0;
    else
    % Calculate the slew rate amplitude produced by the unmodified sample
    % calculate a low pass filtered slew rate
    % Pterm_slew_rate = slew_filter.apply((abs(sample - last_sampleg)/ dt), dt);
    slew_filter_in=abs(sample - last_sample)/ dt;      
    slew_filter=slew_filter + (slew_filter_in - slew_filter) * get_filt_alpha(10);
    Pterm_slew_rate=slew_filter;  
      
    % rectify and apply a decaying envelope filter. The 10 in the
    % constrain limits the modifier to be between 0.1 and 1.0, so we
    % never drop PID gains below 10% of configured value
    alpha = 1.0 - constrain_value(dt/slew_rate_tau, 0.0, 1.0);
    slew_amplitude = constrain_value(Pterm_slew_rate, alpha * slew_amplitude, 10 * slew_rate_max);

    % Calculate the gain adjustment
    mod = slew_rate_max / max(slew_amplitude, slew_rate_max);
    last_sample = mod * sample;

    Dmod= mod;
    end
 
     
    P_out=P_out * Dmod;
    D_out=D_out * Dmod;
    
    output=P_out + integrator + D_out+kff*target_in;
    
    rate_yaw_pid.flags_reset_filter  = flags_reset_filter;
    rate_yaw_pid.target              = target;
    rate_yaw_pid.error               = error;
    rate_yaw_pid.error_last          = error_last ;

    rate_yaw_pid.integrator          = integrator;
    rate_yaw_pid.derivative          = derivative;
    rate_yaw_pid.Dmod                = Dmod;
    rate_yaw_pid.slew_amplitude      = slew_amplitude;
    rate_yaw_pid.slew_filterg        = slew_filter;
    rate_yaw_pid.last_sample         = last_sample;
    
 end

