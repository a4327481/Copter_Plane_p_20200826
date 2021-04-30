function  copter_run_posfree()
%copter run


global AC_PosControl
global SRV_Channel

roll_target                           = AC_PosControl.roll_target;
pitch_target                          = AC_PosControl.pitch_target;
target_yaw_rate                       = AC_PosControl.target_yaw_rate;

tail_tilt                             = SRV_Channel.tail_tilt; 
k_aileron                             = SRV_Channel.k_aileron;
k_elevator                            = SRV_Channel.k_elevator;
k_rudder                              = SRV_Channel.k_rudder;
  
         update_z_controller();
         roll_target=0;
         pitch_target=0;
         target_yaw_rate=0;
         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate);
         rate_controller_run();
         tail_tilt=0;
         k_aileron=0;
         k_elevator=0;
         k_rudder=0;
         AP_Motors_output();
		 
AC_PosControl.roll_target                           = roll_target;
AC_PosControl.pitch_target                          = pitch_target;
AC_PosControl.target_yaw_rate                       = target_yaw_rate;

SRV_Channel.tail_tilt                               = tail_tilt; 
SRV_Channel.k_aileron                               = k_aileron;
SRV_Channel.k_elevator                              = k_elevator;
SRV_Channel.k_rudder                                = k_rudder;		 
		 
		 
                
end

