function  output_to_motors_plane_4a1()

global dt
global AP_Motors
global SRV_Channel

k_throttle               = SRV_Channel.k_throttle;
pwm_out                  = SRV_Channel.pwm_out;
pwm_tail                 = SRV_Channel.pwm_tail;

pwm_max                  = AP_Motors.pwm_max;
pwm_min                  = AP_Motors.pwm_min;
thrust_slew_time         = AP_Motors.thrust_slew_time;
% global mode

    % convert output to PWM and send to each motor
%  switch(mode)
%    case {1,2,3}
%        tail_tilt=0;       
%    case {4,5,6}
%        tail_tilt=-9556;
% %        tail_tilt=0;
%  end     
        thrust_dt=(pwm_max-pwm_min)*dt/thrust_slew_time;
        pwm_out_temp=pwm_min+(pwm_max-pwm_min)*k_throttle;
        pwm_tail=constrain_value(pwm_out_temp,pwm_tail-thrust_dt,pwm_tail+thrust_dt);
        pwm_tail=constrain_value(pwm_tail,pwm_min,pwm_max);
     for i=1:4
        pwm_out_temp=pwm_min;
        pwm_out(i)=constrain_value(pwm_out_temp,pwm_out(i)-thrust_dt,pwm_out(i)+thrust_dt);
        pwm_out(i)=constrain_value(pwm_out(i),pwm_min,pwm_max);
    end
AP_Motors.pwm_max                  = pwm_max;
AP_Motors.pwm_min                  = pwm_min;
AP_Motors.thrust_slew_time         = thrust_slew_time;
SRV_Channel.k_throttle             = k_throttle;
SRV_Channel.pwm_out                = pwm_out;
SRV_Channel.pwm_tail                 = pwm_tail;

end