function Nuo=prevent_indecision( Nu)
 global HD

 global AP_L1
  last_Nu              =AP_L1.last_Nu;
  target_bearing_cd    =AP_L1.target_bearing_cd;
  loiter_direction     =AP_L1.loiter_direction;
%    prevent indecision in our turning by using our previous turn
%    decision if we are in a narrow angle band pointing away from the
%    target and the turn angle has changed sign
        Nu_limit = 0.9*pi;
       
    if(abs(Nu) > Nu_limit &&abs(wrap_PI(target_bearing_cd/HD/100 - get_yaw())) > 12000/HD/100 )
        if(loiter_direction==1)
            Nu=Nu_limit;
        else
            Nu=-Nu_limit;
        end      
    end
     Nuo=Nu;
    if (abs(Nu) > Nu_limit && abs(last_Nu) > Nu_limit &&abs(wrap_PI(target_bearing_cd/HD/100 - get_yaw())) > 12000/HD/100 &&Nu * last_Nu < 0.0)  
        % we are moving away from the target waypoint and pointing
        % away from the waypoint (not flying backwards). The sign
        % of Nu has also changed, which means we are
        % oscillating in our decision about which way to go
        Nuo = last_Nu;
    end
    
  AP_L1.last_Nu              =last_Nu;
  AP_L1.target_bearing_cd    =target_bearing_cd;
  AP_L1.loiter_direction     =loiter_direction;
end

