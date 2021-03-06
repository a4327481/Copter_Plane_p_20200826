function  calc_nav_yaw_coordinated( speed_scaler)
%   calculate yaw control for coordinated flight
  global Test_w
  
  global SRV_Channel
  global Plane
  global Copter_Plane
  k_rudder                  = SRV_Channel.k_rudder;
  k_aileron                 = SRV_Channel.k_aileron;
  kff_rudder_mix            = Plane.kff_rudder_mix;
  disable_AP_yaw_integrator    = Copter_Plane.disable_AP_yaw_integrator;
  
%      rudder_in = rudder_input();
    % Received an external msg that guides yaw in the last 3 seconds?
%     if ((control_mode == &mode_guided || control_mode == &mode_avoidADSB) &&
%             plane.guided_state.last_forced_rpy_ms.z > 0 &&
%             millis() - plane.guided_state.last_forced_rpy_ms.z < 3000)  
%         commanded_rudder = plane.guided_state.forced_rpy_cd.z;
%       else  
%         if (control_mode == &mode_stabilize && rudder_in != 0)  
        commanded_rudder = get_servo_out_yaw(speed_scaler, disable_AP_yaw_integrator);

        % add in rudder mixing from roll
        commanded_rudder =commanded_rudder+ k_aileron * kff_rudder_mix;
        k_rudder = constrain_value(commanded_rudder, -4500, 4500)+Test_w.k_rudder;
        
   SRV_Channel.k_rudder                  = k_rudder;
   SRV_Channel.k_aileron                 = k_aileron;
   Plane.kff_rudder_mix                  = kff_rudder_mix;
   Copter_Plane.disable_AP_yaw_integrator   = disable_AP_yaw_integrator;

end

