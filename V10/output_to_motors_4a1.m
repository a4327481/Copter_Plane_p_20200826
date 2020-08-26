function  output_to_motors_4a1()
global thrust_rpyt_out
global pwm_max
global pwm_min
global pwm_out
global thrust_slew_time
global dt
global pwm_tail
global k_throttle
global tail_tilt
global p_tail_tilt
global mode
global tail_tilt_c2p
    % convert output to PWM and send to each motor
        thrust_dt=(pwm_max-pwm_min)*dt/thrust_slew_time;
        pwm_out_temp=pwm_min+(pwm_max-pwm_min)*k_throttle;
        pwm_tail=constrain_value(pwm_out_temp,pwm_tail-thrust_dt,pwm_tail+thrust_dt);
        pwm_tail=constrain_value(pwm_tail,pwm_min,pwm_max);
    for i=1:4
        pwm_out_temp=pwm_min+(pwm_max-pwm_min)*thrust_rpyt_out(i);
        pwm_out(i)=constrain_value(pwm_out_temp,pwm_out(i)-thrust_dt,pwm_out(i)+thrust_dt);
        pwm_out(i)=constrain_value(pwm_out(i),pwm_min,pwm_max);
    end
