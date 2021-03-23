function  output_to_motors_4a1()

global dt
global mode
global AP_Motors


thrust_rpyt_out             = AP_Motors.thrust_rpyt_out;  
pwm_max                     = AP_Motors.pwm_max;         
pwm_min                     = AP_Motors.pwm_min;         
pwm_out                     = AP_Motors.pwm_out;         
thrust_slew_time            = AP_Motors.thrust_slew_time;
pwm_tail                    = AP_Motors.pwm_tail;        
k_throttle                  = AP_Motors.k_throttle;      
tail_tilt                   = AP_Motors.tail_tilt;       
p_tail_tilt                 = AP_Motors.p_tail_tilt;     
tail_tilt_c2p               = AP_Motors.tail_tilt_c2p;   

    % convert output to PWM and send to each motor
        thrust_dt=(pwm_max-pwm_min)*dt/thrust_slew_time;
        pwm_out_temp=pwm_min+(pwm_max-pwm_min)*k_throttle;
        pwm_tail=constrain_value(pwm_out_temp,pwm_tail-thrust_dt,pwm_tail+thrust_dt);
        pwm_tail=constrain_value(pwm_tail,pwm_min,pwm_max);
    for i=1:4
        if(thrust_rpyt_out(i)<=0.05)
           thrust_rpyt_out(i)=0.05;             
        end
        pwm_out_temp=pwm_min+(pwm_max-pwm_min)*thrust_rpyt_out(i);
        pwm_out(i)=constrain_value(pwm_out_temp,pwm_out(i)-thrust_dt,pwm_out(i)+thrust_dt);
        pwm_out(i)=constrain_value(pwm_out(i),pwm_min,pwm_max);
    end
	
AP_Motors.thrust_rpyt_out             = thrust_rpyt_out;  
AP_Motors.pwm_max                     = pwm_max;         
AP_Motors.pwm_min                     = pwm_min;         
AP_Motors.pwm_out                     = pwm_out;         
AP_Motors.thrust_slew_time            = thrust_slew_time;
AP_Motors.pwm_tail                    = pwm_tail;        
AP_Motors.k_throttle                  = k_throttle;      
AP_Motors.tail_tilt                   = tail_tilt;       
AP_Motors.p_tail_tilt                 = p_tail_tilt;     
AP_Motors.tail_tilt_c2p               = tail_tilt_c2p;
	
end