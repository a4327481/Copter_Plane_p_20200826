function  update_speed(load_factor)
 global dt
 global AP_TECS
 global Plane
 global SINS
 
 aspeed              = SINS.aspeed;
 EAS2TAS             = SINS.EAS2TAS;
 TAS_dem             = AP_TECS.TAS_dem;
 TASmax              = AP_TECS.TASmax;
 TASmin              = AP_TECS.TASmin;
 EAS_dem             = AP_TECS.EAS_dem;
 TAS_state           = AP_TECS.TAS_state;
 spdCompFiltOmega    = AP_TECS.spdCompFiltOmega;
 integDTAS_state     = AP_TECS.integDTAS_state;
 vel_dot             = AP_TECS.vel_dot;
 airspeed_max        = Plane.airspeed_max;
 airspeed_min        = Plane.airspeed_min;
    % Calculate time in seconds since last update
    % Convert equivalent airspeeds to true airspeeds
    EAS=aspeed;
    TAS_dem = EAS_dem * EAS2TAS;
    TASmax   = airspeed_max * EAS2TAS;
    TASmin   = airspeed_min * EAS2TAS;
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        TASmin =TASmin* load_factor;
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (TASmax < TASmin)  
        TASmax = TASmin;
    end
    if (TASmin > TAS_dem)  
        TASmin = TAS_dem;
    end

    % Reset states of time since last update is too large
     

    % Get airspeed or default to halfway between min and max if
    % airspeed is not being used and set speed rate to zero
  
  
    % Implement a second order complementary filter to obtain a
    % smoothed airspeed estimate
    % airspeed estimate is held in _TAS_state
      aspdErr = ( EAS * EAS2TAS) - TAS_state;
      integDTAS_input = aspdErr * spdCompFiltOmega * spdCompFiltOmega;
    % Prevent state from winding up
    if (TAS_state < 3.1)  
        integDTAS_input = max(integDTAS_input , 0.0);
    end
    integDTAS_state = integDTAS_state + integDTAS_input * dt;
    TAS_input = integDTAS_state + vel_dot + aspdErr * spdCompFiltOmega * 1.4142;
    TAS_state = TAS_state + TAS_input * dt;
    % limit the airspeed to a minimum of 3 m/s
    TAS_state = max(TAS_state, 3.0);
    
    
    
 AP_TECS.TAS_dem             = TAS_dem;
 AP_TECS.TASmax              = TASmax;
 AP_TECS.TASmin              = TASmin;
 AP_TECS.EAS_dem             = EAS_dem;
 AP_TECS.TAS_state           = TAS_state;
 AP_TECS.spdCompFiltOmega    = spdCompFiltOmega;
 AP_TECS.integDTAS_state     = integDTAS_state;
 AP_TECS.vel_dot             = vel_dot;
end

