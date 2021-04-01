function  update_throttle_with_airspeed()
global dt
global AP_TECS
global SINS

 rot_body_to_ned          = SINS.rot_body_to_ned;
 TASmax                   = AP_TECS.TASmax;                
 SKE_dem                  = AP_TECS.SKE_dem;            
 TASmin                   = AP_TECS.TASmin;             
 SPE_dem                  = AP_TECS.SPE_dem;            
 SPE_est                  = AP_TECS.SPE_est;            
 SKE_est                  = AP_TECS.SKE_est;            
 STE_error                = AP_TECS.STE_error;          
 SPEdot_dem               = AP_TECS.SPEdot_dem;         
 SKEdot_dem               = AP_TECS.SKEdot_dem;         
 STEdot_min               = AP_TECS.STEdot_min;        
 STEdot_max               = AP_TECS.STEdot_max;         
 SPEdot                   = AP_TECS.SPEdot;             
 SKEdot                   = AP_TECS.SKEdot;             
 STEdotErrLast            = AP_TECS.STEdotErrLast;      
 timeConstant             = AP_TECS.timeConstant;       
 throttle_cruise          = AP_TECS.throttle_cruise;    
 rollComp                 = AP_TECS.rollComp;           
 THRmaxf                  = AP_TECS.THRmaxf;            
 THRminf                  = AP_TECS.THRminf;            
 thrDamp                  = AP_TECS.thrDamp;            
 throttle_dem             = AP_TECS.throttle_dem;       
 throttle_slewrate        = AP_TECS.throttle_slewrate;  
 last_throttle_dem        = AP_TECS.last_throttle_dem; 
 integGain                = AP_TECS.integGain;          
 integTHR_state           = AP_TECS.integTHR_state;     
 p_ff_throttle            = AP_TECS.p_ff_throttle;      
 ff_throttle              = AP_TECS.ff_throttle;        
 spdWeight                = AP_TECS.spdWeight;          
 % calculate throttle demand - airspeed enabled case
 
 
    % Calculate limits to be applied to potential energy error to prevent over or underspeed occurring due to large height errors
      SPE_err_max = 0.5  * TASmax * TASmax - SKE_dem;
      SPE_err_min = 0.5  * TASmin * TASmin - SKE_dem;
    
    % Calculate total energy error
    STE_error = constrain_value((SPE_dem - SPE_est), SPE_err_min, SPE_err_max) + SKE_dem - SKE_est;
    STEdot_dem = constrain_value((SPEdot_dem + SKEdot_dem), STEdot_min, STEdot_max);
    STEdot_error = STEdot_dem - SPEdot - SKEdot;

    % Apply 0.5 second first order filter to STEdot_error
    % This is required to remove accelerometer noise from the  measurement
    STEdot_error = 0.2*STEdot_error + 0.8*STEdotErrLast;
    STEdotErrLast = STEdot_error;

    % Calculate throttle demand
    % If underspeed condition is set, then demand full throttle
     
        % Calculate gain scaler from specific energy error to throttle
        % (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf) is the derivative of STEdot wrt throttle measured across the max allowed throttle range.
        K_STE2Thr = 1 / (timeConstant * (STEdot_max - STEdot_min) / (THRmaxf - THRminf));
        
        % Calculate feed-forward throttle
%         ff_throttle = 0;
        nomThr = throttle_cruise * 0.01;
%          Use the demanded rate of change of total energy as the feed-forward demand, but add
%         additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
%         drag increase during turns.
        cosPhi = sqrt((rot_body_to_ned(1,2)*rot_body_to_ned(1,2)) + (rot_body_to_ned(2,2)*rot_body_to_ned(2,2)));
        STEdot_dem = STEdot_dem + rollComp * (1.0/constrain_value(cosPhi * cosPhi , 0.1, 1.0) - 1.0);
        ff_throttle = nomThr + STEdot_dem / (STEdot_max - STEdot_min) * (THRmaxf - THRminf)*p_ff_throttle;

        % Calculate PD + FF throttle
         throttle_damp = thrDamp;

        throttle_dem = (STE_error + STEdot_error * throttle_damp) * K_STE2Thr + ff_throttle;

        % Constrain throttle demand
        throttle_dem = constrain_value(throttle_dem, THRminf, THRmaxf);

         THRminf_clipped_to_zero = constrain_value(THRminf, 0, THRmaxf);

        % Rate limit PD + FF throttle
        % Calculate the throttle increment from the specified slew time
        if (throttle_slewrate ~= 0)  
             thrRateIncr = dt * (THRmaxf - THRminf_clipped_to_zero) * throttle_slewrate * 0.01;

            throttle_dem = constrain_value(throttle_dem,last_throttle_dem - thrRateIncr,last_throttle_dem + thrRateIncr);
            last_throttle_dem = throttle_dem;
        end

        % Calculate integrator state upper and lower limits
        % Set to a value that will allow 0.1 (10%) throttle saturation to allow for noise on the demand
        % Additionally constrain the integrator state amplitude so that the integrator comes off limits faster.
         maxAmp = 0.5*(THRmaxf - THRminf_clipped_to_zero);
         integ_max = constrain_value((THRmaxf - throttle_dem + 0.1),-maxAmp,maxAmp);
         integ_min = constrain_value((THRminf - throttle_dem - 0.1),-maxAmp,maxAmp);

        % Calculate integrator state, constraining state
        % Set integrator to a max throttle value during climbout
        integTHR_state = integTHR_state + (STE_error * integGain) * dt * K_STE2Thr;              
        integTHR_state = constrain_value(integTHR_state, integ_min, integ_max);
         

        % Sum the components.
        throttle_dem = throttle_dem + integTHR_state;
    % Constrain throttle demand
    throttle_dem = constrain_value(throttle_dem, THRminf, THRmaxf);
    if(spdWeight==2)
        throttle_dem=0;
        last_throttle_dem=throttle_dem;
    end
 
    AP_TECS.TASmax                        = TASmax;
    AP_TECS.SKE_dem                       = SKE_dem;
    AP_TECS.TASmin                        = TASmin;
    AP_TECS.SPE_dem                       = SPE_dem;
    AP_TECS.SPE_est                       = SPE_est;
    AP_TECS.SKE_est                       = SKE_est;
    AP_TECS.STE_error                     = STE_error;
    AP_TECS.SPEdot_dem                    = SPEdot_dem;
    AP_TECS.SKEdot_dem                    = SKEdot_dem;
    AP_TECS.STEdot_min                    = STEdot_min;
    AP_TECS.STEdot_max                    = STEdot_max;
    AP_TECS.SPEdot                        = SPEdot;
    AP_TECS.SKEdot                        = SKEdot;
    AP_TECS.STEdotErrLast                 = STEdotErrLast;
    AP_TECS.timeConstant                  = timeConstant;
    AP_TECS.throttle_cruise               = throttle_cruise;
    AP_TECS.rollComp                      = rollComp;
    AP_TECS.THRmaxf                       = THRmaxf;
    AP_TECS.THRminf                       = THRminf;
    AP_TECS.thrDamp                       = thrDamp;
    AP_TECS.throttle_dem                  = throttle_dem;
    AP_TECS.throttle_slewrate             = throttle_slewrate;
    AP_TECS.last_throttle_dem             = last_throttle_dem;
    AP_TECS.integGain                     = integGain;
    AP_TECS.integTHR_state                = integTHR_state;
    AP_TECS.p_ff_throttle                 = p_ff_throttle;
    AP_TECS.ff_throttle                   = ff_throttle;
    AP_TECS.spdWeight                     = spdWeight;
 
 
 
 
 
 
 
end

