function  output_to_motors()
global AP_Motors

global dt
global Copter_Plane
global SRV_Channel 

mode                      = Copter_Plane.mode; 
pwm_out                   = SRV_Channel.pwm_out;
tail_tilt                 = SRV_Channel.tail_tilt;

thrust_rpyt_out           = AP_Motors.thrust_rpyt_out;
pwm_max                   = AP_Motors.pwm_max;
pwm_min                   = AP_Motors.pwm_min;
thrust_slew_time          = AP_Motors.thrust_slew_time;
p_tail_tilt               = AP_Motors.p_tail_tilt;
tail_tilt_c2p             = AP_Motors.tail_tilt_c2p;


    % convert output to PWM and send to each motor
    thrust_dt=(pwm_max-pwm_min)*dt/thrust_slew_time;
    

switch(mode)
   case {1,2,3}
       tail_tilt=0;       
   case {4,5,6}
       tail_tilt=-9556;
%        tail_tilt=0;
end
    tail_tilt_temp=constrain_value(tail_tilt,tail_tilt_c2p,700);
    thrust_rpyt_out([2 4])=thrust_rpyt_out([2 4])*((1/cosd(tail_tilt_temp/100)-1)*p_tail_tilt+1);
    for i=1:4
        pwm_out_temp=pwm_min+(pwm_max-pwm_min)*thrust_rpyt_out(i);
        pwm_out(i)=constrain_value(pwm_out_temp,pwm_out(i)-thrust_dt,pwm_out(i)+thrust_dt);
        pwm_out(i)=constrain_value(pwm_out(i),pwm_min,pwm_max);
    end
	
AP_Motors.thrust_rpyt_out           = thrust_rpyt_out;
AP_Motors.pwm_max                   = pwm_max;
AP_Motors.pwm_min                   = pwm_min;
AP_Motors.thrust_slew_time          = thrust_slew_time;
AP_Motors.p_tail_tilt               = p_tail_tilt;
AP_Motors.tail_tilt_c2p             = tail_tilt_c2p;

SRV_Channel.pwm_out                  = pwm_out;
SRV_Channel.tail_tilt                 = tail_tilt;
   
end
