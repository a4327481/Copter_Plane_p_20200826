function  pwm_output=output_to_pwm(  actuator)
global spool_state
global disarm_disable_pwm
global armed
global pwm_min
global pwm_max

if (spool_state == SpoolState.SHUT_DOWN)
    % in shutdown mode, use PWM 0 or minimum PWM
    if (disarm_disable_pwm && ~armed)
        pwm_output = 0;
    else
        pwm_output = pwm_min;
    end
else
    % in all other spool modes, covert to desired PWM
    pwm_output = pwm_min + (pwm_max - pwm_min) * actuator;
end
end

