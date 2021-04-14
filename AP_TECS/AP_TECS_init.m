function  AP_TECS_init()
 global AP_TECS 
 global SRV_Channel
 global SINS
    % Initialise states and variables if DT > 1 second or in climbout
 
        AP_TECS.integTHR_state      = 0.0;
        AP_TECS.integSEB_state      = 0.0;
        AP_TECS.last_throttle_dem = SRV_Channel.k_throttle;
        AP_TECS.last_pitch_dem    = SINS.pitch;
        AP_TECS.hgt_dem_adj_last  = AP_TECS.hgt_dem_cm*0.01;
        AP_TECS.hgt_dem_adj       = AP_TECS.hgt_dem_cm*0.01;
        AP_TECS.hgt_dem_prev      = AP_TECS.hgt_dem_cm*0.01;
        AP_TECS.hgt_dem_in_old    = AP_TECS.hgt_dem_cm*0.01;
        AP_TECS.TAS_dem_adj       = AP_TECS.EAS_dem * AP_TECS.EAS2TAS;
       end