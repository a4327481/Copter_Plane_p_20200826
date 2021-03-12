function  copter_run()
%copter run
global roll_target
global pitch_target
global target_yaw_rate
global tail_tilt
global k_aileron
global k_elevator
global k_rudder
global aspeed
global aspeed_cp
global nav_pitch_cd
global nav_roll_cd
global p_plane_cp
global AC_PosControl
vel_desired             = AC_PosControl.vel_desired;


         if( (abs(vel_desired(1))>0) || (abs(vel_desired(2))>0)||(abs(target_yaw_rate)>0))
             temp_yaw_rate=0;
         else
             temp_yaw_rate=get_weathervane_yaw_rate_cds();
         end   
         vel_forward_pct_out = forward_throttle_pctg();%%tail xiu zheng
         update_vel_controller_xy();
         update_z_controller();
         input_euler_angle_roll_pitch_euler_rate_yaw(  roll_target,   pitch_target,   target_yaw_rate+temp_yaw_rate);
         rate_controller_run();
         tail_tilt=vel_forward_pct_out;
                  
         if(aspeed>aspeed_cp)
            nav_pitch_cd=pitch_target;
            nav_roll_cd=roll_target;
            stabilize()
            k_aileron=k_aileron*p_plane_cp;
            k_elevator=k_elevator*p_plane_cp;
            k_rudder=k_rudder*p_plane_cp;            
         else          
             k_aileron=0;
             k_elevator=0;
             k_rudder=0;
         end
         

         AP_MotorsMulticopter_output();
                
end

