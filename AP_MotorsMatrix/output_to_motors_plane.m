function  output_to_motors_plane()
global dt
global AP_Motors

pwm_max                        = AP_Motors.pwm_max;
pwm_min                        = AP_Motors.pwm_min;
thrust_slew_time               = AP_Motors.thrust_slew_time;

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
SRV_Channel.pwm_out                      = pwm_out;


end