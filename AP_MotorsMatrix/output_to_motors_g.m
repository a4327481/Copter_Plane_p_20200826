function  output_to_motors_g()
global actuator
global spool_state
global thrust_rpyt_out
global pwm_out
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

end

