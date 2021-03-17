function  output_to_motors_plane()
global pwm_max
global pwm_min
global thrust_slew_time
global dt

global SRV_Channel
k_throttle     = SRV_Channel.k_throttle;
pwm_out        = SRV_Channel.pwm_out;
tail_tilt      = SRV_Channel.tail_tilt;

        tail_tilt=-9556;
        thrust_dt=(pwm_max-pwm_min)*dt/thrust_slew_time;
    for i=2:2:4
        pwm_out_temp=pwm_min+(pwm_max-pwm_min)*k_throttle;
        pwm_out(i)=constrain_value(pwm_out_temp,pwm_out(i)-thrust_dt,pwm_out(i)+thrust_dt);
        pwm_out(i)=constrain_value(pwm_out(i),pwm_min,pwm_max);
    end
    for i=1:2:3
        pwm_out_temp=pwm_min;
        pwm_out(i)=constrain_value(pwm_out_temp,pwm_out(i)-thrust_dt,pwm_out(i)+thrust_dt);
        pwm_out(i)=constrain_value(pwm_out(i),pwm_min,pwm_max);
    end
    
    
    SRV_Channel.tail_tilt        =tail_tilt;

    
end