function  calc_nav_pitch()
global HD
global AP_TECS
global Plane
 % calculate a new nav_pitch_cd from the speed height controller


pitch_dem                        = AP_TECS.pitch_dem;
pitch_limit_max_cd               = Plane.pitch_limit_max_cd;
pitch_limit_min_cd               = Plane.pitch_limit_min_cd;
nav_pitch_cd                     = Plane.nav_pitch_cd;
% Calculate the Pitch of the plane
    % --------------------------------
      commanded_pitch = pitch_dem *HD*100;

%     % Received an external msg that guides roll in the last 3 seconds?
%     if ((control_mode == &mode_guided || control_mode == &mode_avoidADSB) &&
%             plane.guided_state.last_forced_rpy_ms.y > 0 &&
%             millis() - plane.guided_state.last_forced_rpy_ms.y < 3000)  
%         commanded_pitch = plane.guided_state.forced_rpy_cd.y;
     

    nav_pitch_cd = constrain_value(commanded_pitch, pitch_limit_min_cd, pitch_limit_max_cd);
 
   Plane.nav_pitch_cd     = nav_pitch_cd;
   
end

