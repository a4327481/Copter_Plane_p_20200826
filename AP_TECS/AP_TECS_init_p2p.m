function  AP_TECS_init_p2p()
 global AP_TECS 
 global SRV_Channel
 global SINS
 global Copter_Plane
    % Initialise states and variables if DT > 1 second or in climbout
 
%         AP_TECS.integTHR_state      = 0.0;
%         AP_TECS.integSEB_state      = 0.0;
%         AP_TECS.last_throttle_dem = SRV_Channel.k_throttle;
%         AP_TECS.last_pitch_dem    = SINS.pitch;
        AP_TECS.hgt_dem_adj_last  = Copter_Plane.hgt_dem_cm*0.01;
        AP_TECS.hgt_dem_adj       = Copter_Plane.hgt_dem_cm*0.01;
        AP_TECS.hgt_dem_prev      = Copter_Plane.hgt_dem_cm*0.01;
        AP_TECS.hgt_dem_in_old    = Copter_Plane.hgt_dem_cm*0.01;
%         AP_TECS.TAS_dem_adj       = AP_TECS.EAS_dem * SINS.EAS2TAS;
       end