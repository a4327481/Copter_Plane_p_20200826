function output=pid_accel_xy_update_all( target_in,  measurement,  limit)
global dt
global AC_PosControl

kp                                  =AC_PosControl.pid_vel_xy.kp;
ki                                  =AC_PosControl.pid_vel_xy.ki;
kd                                  =AC_PosControl.pid_vel_xy.kd;
kff                                 =AC_PosControl.pid_vel_xy.kff;
kimax                               =AC_PosControl.pid_vel_xy.kimax;
filt_T_hz                           =AC_PosControl.pid_vel_xy.filt_T_hz;
filt_E_hz                           =AC_PosControl.pid_vel_xy.filt_E_hz;
filt_D_hz                           =AC_PosControl.pid_vel_xy.filt_D_hz;
% slew_rate_max                       =AC_PosControl.pid_vel_xy.slew_rate_max;
% slew_rate_tau                       =AC_PosControl.pid_vel_xy.slew_rate_tau;

flags_reset_filter                  =AC_PosControl.pid_vel_xy.flags_reset_filter;
disable_integrator                  =AC_PosControl.pid_vel_xy.disable_integrator;
target                              =AC_PosControl.pid_vel_xy.target;
error                               =AC_PosControl.pid_vel_xy.error;
error_last                          =AC_PosControl.pid_vel_xy.error_last;
integrator                          =AC_PosControl.pid_vel_xy.integrator;
derivative                          =AC_PosControl.pid_vel_xy.derivative;
% slew_amplitude                      =AC_PosControl.pid_vel_xy.slew_amplitude;
% slew_filter                         =AC_PosControl.pid_vel_xy.slew_filterg;
% last_sample                         =AC_PosControl.pid_vel_xy.last_sample;
% Dmod                                =AC_PosControl.pid_vel_xy.Dmod;
% don't process inf or NaN

% reset input filter to value received
if (flags_reset_filter)
    flags_reset_filter = false;
    target = target_in;
    error = target - measurement;
    derivative = [0 0];
    integrator = [0 0];
    %         slew_filter=0;
    %         last_sample=error * kp+derivative * kd;
else
    error_last = error;
    target=target + get_filt_alpha(filt_T_hz) * (target_in - target);
    error =error  + get_filt_alpha(filt_E_hz) * ((target - measurement) - error);
end
% calculate and filter derivative
if (dt > 0.0)
    derivative_in = (error - error_last) / dt;
    derivative=derivative + get_filt_alpha(filt_D_hz) * (derivative_in - derivative);
end
% update I term
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (ki~=0 && ~disable_integrator )
    % Ensure that integrator can only be reduced if the output is saturated
    if (~limit )
        integrator = integrator + (error * ki) * dt;
        integrator_length=norm(integrator,2);
        if ((integrator_length > kimax) && (integrator_length>0))
            integrator=integrator *kimax / integrator_length;
        end
    else
        integrator_length_orig = min(norm(integrator,2), kimax);
        integrator=integrator + (error * ki) * dt;
        integrator_length_new = norm(integrator,2);
        if ((integrator_length_new > integrator_length_orig) && (integrator_length_new>0))
            integrator=integrator * (integrator_length_orig / integrator_length_new);
        end
    end
else
    integrator = [0  0];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P_out = (error * kp);
D_out = (derivative * kd);

%     % calculate slew limit modifier for P+D
% %     Dmodg = AC_PosControl.pid_accel_z_modifier(P_out + D_out, dt);
%     sample=P_out + D_out;
% %   apply filter to sample, returning multiplier between 0 and 1 to keep
% %   output within slew rate
%
% %    modifier(float sample, float dt)
%
%     if (slew_rate_max <= 0)
%         Dmod= 1.0;
%     else
%     % Calculate the slew rate amplitude produced by the unmodified sample
%     % calculate a low pass filtered slew rate
%     % Pterm_slew_rate = slew_filter.apply((abs(sample - last_sampleg)/ dt), dt);
%     slew_filter_in=abs(sample - last_sample)/ dt;
%     slew_filter=slew_filter + (slew_filter_in - slew_filter) * get_filt_alpha(10);
%     Pterm_slew_rate=slew_filter;
%
%     % rectify and apply a decaying envelope filter. The 10 in the
%     % constrain limits the modifier to be between 0.1 and 1.0, so we
%     % never drop PID gains below 10% of configured value
%     alpha = 1.0 - constrain_value(dt/slew_rate_tau, 0.0, 1.0);
%     slew_amplitude = constrain_value(Pterm_slew_rate, alpha * slew_amplitude, 10 * slew_rate_max);
%
%     % Calculate the gain adjustment
%     mod = slew_rate_max / max(slew_amplitude, slew_rate_max);
%     last_sample = mod * sample;
%
%     Dmod= mod;
%     end
%
%
%     P_out=P_out * Dmod;
%     D_out=D_out * Dmod;

output=P_out + integrator + D_out+kff*target_in;

AC_PosControl.pid_vel_xy.flags_reset_filter  = flags_reset_filter;
AC_PosControl.pid_vel_xy.disable_integrator  = disable_integrator;

AC_PosControl.pid_vel_xy.target              = target;
AC_PosControl.pid_vel_xy.error               = error;
AC_PosControl.pid_vel_xy.error_last          = error_last ;

AC_PosControl.pid_vel_xy.integrator          = integrator;
AC_PosControl.pid_vel_xy.derivative          = derivative;
%     AC_PosControl.pid_vel_xy.Dmod                = Dmod;
%     AC_PosControl.pid_vel_xy.slew_amplitude      = slew_amplitude;
%     AC_PosControl.pid_vel_xy.slew_filterg        = slew_filter;
%     AC_PosControl.pid_vel_xy.last_sample         = last_sample;

end

