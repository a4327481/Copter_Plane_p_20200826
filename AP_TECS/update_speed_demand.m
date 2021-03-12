function   update_speed_demand( )
global  dt
global  AP_TECS

TAS_dem                     = AP_TECS.TAS_dem;
TASmin                      = AP_TECS.TASmin;
TASmax                      = AP_TECS.TASmax;
STEdot_max                  = AP_TECS.STEdot_max;
STEdot_min                  = AP_TECS.STEdot_min;
TAS_state                   = AP_TECS.TAS_state;
TAS_dem_adj                 = AP_TECS.TAS_dem_adj;
TAS_rate_dem                = AP_TECS.TAS_rate_dem;


    % Set the airspeed demand to the minimum value if an underspeed condition exists
    % or a bad descent condition exists
    % This will minimise the rate of descent resulting from an engine failure,
    % enable the maximum climb rate to be achieved and prevent continued full power descent
    % into the ground due to an unachievable airspeed value
 

    % Constrain speed demand, taking into account the load factor
    TAS_dem = constrain_value(TAS_dem, TASmin, TASmax);

    % calculate velocity rate limits based on physical performance limits
    % provision to use a different rate limit if bad descent or underspeed condition exists
    % Use 50% of maximum energy rate to allow margin for total energy contgroller
      velRateMax = 0.5 * STEdot_max / TAS_state;
      velRateMin = 0.5 * STEdot_min / TAS_state;
      TAS_dem_previous = TAS_dem_adj;

%     % assume fixed 10Hz call rate
%       dt = 0.1;

    % Apply rate limit
    if ((TAS_dem - TAS_dem_previous) > (velRateMax * dt))
        TAS_dem_adj = TAS_dem_previous + velRateMax * dt;
        TAS_rate_dem = velRateMax;
    elseif ((TAS_dem - TAS_dem_previous) < (velRateMin * dt))
        TAS_dem_adj = TAS_dem_previous + velRateMin * dt;
        TAS_rate_dem = velRateMin;     
    else
        TAS_rate_dem = (TAS_dem - TAS_dem_previous) / dt;
        TAS_dem_adj = TAS_dem;
    end
    % Constrain speed demand again to protect against bad values on initialisation.
    TAS_dem_adj = constrain_value(TAS_dem_adj, TASmin, TASmax);
    
AP_TECS.TAS_dem                     = TAS_dem;
AP_TECS.TASmin                      = TASmin;
AP_TECS.STEdot_max                  = STEdot_max;
AP_TECS.STEdot_min                  = STEdot_min;
AP_TECS.TAS_state                   = TAS_state;
AP_TECS.TAS_dem_adj                 = TAS_dem_adj;
AP_TECS.TAS_rate_dem                = TAS_rate_dem;
    
    
end

