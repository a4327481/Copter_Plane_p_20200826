function  output_to_motors_g()
global AP_Motors
global SRV_Channel
global dt
actuator                     = AP_Motors.actuator;
spool_state                  = AP_Motors.spool_state;
thrust_rpyt_out              = AP_Motors.thrust_rpyt_out;
pwm_max                      = AP_Motors.pwm_max;         
pwm_min                      = AP_Motors.pwm_min;         
thrust_slew_time             = AP_Motors.thrust_slew_time;

pwm_out                      = SRV_Channel.pwm_out;
pwm_tail                     = SRV_Channel.pwm_tail;
k_throttle                   = SRV_Channel.k_throttle;

switch (spool_state)
    case SpoolState.SHUT_DOWN
        % no output
        actuator(1:4)=0;
    case SpoolState.GROUND_IDLE
        % sends output to motors when armed but not flying
        for i = 1:4
            actuator(i)=set_actuator_with_slew(actuator(i), actuator_spin_up_to_ground_idle());
        end
    case {SpoolState.SPOOLING_UP,SpoolState.THROTTLE_UNLIMITED,SpoolState.SPOOLING_DOWN}
        % set motor output based on thrust requests
        for i=1:4
             actuator(i)=set_actuator_with_slew(actuator(i), thrust_to_actuator(thrust_rpyt_out(i)));
%              actuator(i)=set_actuator_with_slew(actuator(i), (thrust_rpyt_out(i)));

        end
end
% convert output to PWM and send to each motor
for i=1:4
    pwm_out(i)=output_to_pwm(actuator(i));
end

thrust_dt=(pwm_max-pwm_min)*dt/thrust_slew_time;
pwm_out_temp=pwm_min+(pwm_max-pwm_min)*k_throttle;
pwm_tail=constrain_value(pwm_out_temp,pwm_tail-thrust_dt,pwm_tail+thrust_dt);
pwm_tail=constrain_value(pwm_tail,pwm_min,pwm_max);
        
AP_Motors.actuator                     = actuator;
AP_Motors.spool_state                  = spool_state;
AP_Motors.thrust_rpyt_out              = thrust_rpyt_out;
SRV_Channel.pwm_out                    = pwm_out;
SRV_Channel.pwm_tail                   = pwm_tail;
end

