function  copter_run_posfree()
%copter run
global roll_target
global pitch_target
global target_yaw_rate
global tail_tilt
global k_aileron
global k_elevator
global k_rudder
global vel_desired
global aspeed
global aspeed_cp
global nav_pitch_cd
global nav_roll_cd
global p_plane_cp
  
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
         AP_MotorsMulticopter_output();
                
end

