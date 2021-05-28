function [output ,AC_PID] =AC_PID_update_error(  error_in, limit,AC_PID)
global dt

kp                      =AC_PID.kp;
ki                      =AC_PID.ki;
kd                      =AC_PID.kd;
kff                     =AC_PID.kff;
kimax                   =AC_PID.kimax;
filt_T_hz               =AC_PID.filt_T_hz;
filt_E_hz               =AC_PID.filt_E_hz;
filt_D_hz               =AC_PID.filt_D_hz;
slew_rate_max           =AC_PID.slew_rate_max;
slew_rate_tau           =AC_PID.slew_rate_tau;

flags_reset_filter      =AC_PID.flags_reset_filter;
disable_integrator      =AC_PID.disable_integrator;
target                  =AC_PID.target;
error                   =AC_PID.error;
error_last              =AC_PID.error_last;
integrator              =AC_PID.integrator;
derivative              =AC_PID.derivative;
slew_amplitude          =AC_PID.slew_amplitude;
slew_filter             =AC_PID.slew_filterg;
last_sample             =AC_PID.last_sample;
Dmod                    =AC_PID.Dmod;



% don't process inf or NaN

target = 0.0;

% reset input filter to value received
if ( flags_reset_filter)
    flags_reset_filter = false;
    error = error_in;
    derivative = 0.0;
    integrator = 0;
    slew_filter=0;
    last_sample=error * kp+derivative * kd;
else
    error_last = error;
    error=error + get_filt_alpha(filt_E_hz) * (error_in - error);
    
    % calculate and filter derivative
    if (dt > 0.0)
         derivative_in = (error - error_last) / dt;
         derivative =derivative+ get_filt_alpha(filt_D_hz) * (derivative_in - derivative);
    end
end

% update I term
if (ki~=0 && ~disable_integrator )
    % Ensure that integrator can only be reduced if the output is saturated
    if (~limit || (((integrator>0) && (error<0)) || ((integrator<0) && (error>0))))
        integrator = integrator + (error * ki) * dt;
        integrator = constrain_value(integrator, -kimax, kimax);
    end
else
    integrator = 0.0;
end



 P_out = (error * kp);
 D_out = (derivative * kd);

% calculate slew limit modifier for P+D
% pid_info.Dmod = slew_limiter.modifier(pid_info.P + pid_info.D, dt);

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
    
    output=P_out + integrator + D_out;

AC_PID.flags_reset_filter  = flags_reset_filter;
AC_PID.disable_integrator  = disable_integrator;
AC_PID.target              = target;
AC_PID.error               = error;
AC_PID.error_last          = error_last ;

AC_PID.integrator          = integrator;
AC_PID.derivative          = derivative;
AC_PID.Dmod                = Dmod;
AC_PID.slew_amplitude      = slew_amplitude;
AC_PID.slew_filterg        = slew_filter;
AC_PID.last_sample         = last_sample;
end

