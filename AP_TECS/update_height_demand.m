function  update_height_demand( )
 global dt
 global AP_TECS
 global Copter_Plane
   hgt_dem                    =AP_TECS.hgt_dem;
   hgt_dem_in_old             =AP_TECS.hgt_dem_in_old;
   maxSinkRate                =AP_TECS.maxSinkRate;
   hgt_dem_prev               =AP_TECS.hgt_dem_prev;
   maxClimbRate               =AP_TECS.maxClimbRate;
   hgt_dem_adj                =AP_TECS.hgt_dem_adj;
   hgt_dem_adj_last           =AP_TECS.hgt_dem_adj_last;
   hgt_rate_dem               =AP_TECS.hgt_rate_dem;
   inint_hgt                  = Copter_Plane.inint_hgt;

 
    if(inint_hgt)
       hgt_dem_in_old=hgt_dem; 
       hgt_dem_prev=hgt_dem;
       hgt_dem_adj_last=hgt_dem;
       inint_hgt=0;   
    end
    % Apply 2 point moving average to demanded height
    hgt_dem = 0.5 * (hgt_dem + hgt_dem_in_old);
    hgt_dem_in_old = hgt_dem;

%     if (maxSinkRate_approach > 0 && _flags.is_doing_auto_land)  
%         % special sink rate for approach to accommodate steep slopes and reverse thrust.
%         % A special check must be done to see if we're LANDing on approach but also if
%         % we're in that tiny window just starting NAV_LAND but still in NORMAL mode. If
%         % we have a steep slope with a short approach we'll want to allow acquiring the
%         % glide slope right away.
%         max_sink_rate = _maxSinkRate_approach;
%     end
    
     
    % Limit height rate of change
    if ((hgt_dem - hgt_dem_prev) > (maxClimbRate * dt))
     
        hgt_dem = hgt_dem_prev + maxClimbRate * dt;
    
    elseif ((hgt_dem - hgt_dem_prev) < (-maxSinkRate * dt))
     
        hgt_dem = hgt_dem_prev - maxSinkRate * dt;
    end
     hgt_dem_prev =  hgt_dem;
    % Apply first order lag to height demand
     hgt_dem_adj = 0.05  *  hgt_dem + 0.95  *  hgt_dem_adj_last;
    % when flaring force height rate demand to the
    % configured sink rate and adjust the demanded height to
    % be kinematically consistent with the height rate. 
        hgt_rate_dem = (hgt_dem_adj - hgt_dem_adj_last) / dt;
     % for landing approach we will predict ahead by the time constant
    % plus the lag produced by the first order filter. This avoids a
    % lagged height demand while constantly descending which causes
    % us to consistently be above the desired glide slope. This will
    % be replaced with a better zero-lag filter in the future.
         hgt_dem_adj_last =  hgt_dem_adj;
         
   AP_TECS.hgt_dem                    = hgt_dem;
   AP_TECS.hgt_dem_in_old             = hgt_dem_in_old;
   AP_TECS.maxSinkRate                = maxSinkRate;
   AP_TECS.hgt_dem_prev               = hgt_dem_prev;
   AP_TECS.maxClimbRate               = maxClimbRate;
   AP_TECS.hgt_dem_adj                = hgt_dem_adj;
   AP_TECS.hgt_dem_adj_last           = hgt_dem_adj_last;
   AP_TECS.hgt_rate_dem               = hgt_rate_dem;
   Copter_Plane.inint_hgt             = inint_hgt;
      
end

