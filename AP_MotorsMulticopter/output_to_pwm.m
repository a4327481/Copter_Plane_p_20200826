function  pwm_output=output_to_pwm(  actuator)
 global AP_Motors	
 global Copter_Plane
 
armed                          = Copter_Plane.armed;                      
spool_state                    = AP_Motors.spool_state;
disarm_disable_pwm             = AP_Motors.disarm_disable_pwm;
pwm_min                        = AP_Motors.pwm_min;
pwm_max                        = AP_Motors.pwm_max;

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
AP_Motors.spool_state                    = spool_state;
AP_Motors.disarm_disable_pwm             = disarm_disable_pwm;
AP_Motors.pwm_min                        = pwm_min;
AP_Motors.pwm_max                        = pwm_max;
Copter_Plane.armed                       = armed;                      

end

