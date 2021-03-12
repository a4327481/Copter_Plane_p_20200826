function update_pitch()
 
    % Calculate Speed/Height Control Weighting
    % This is used to determine how the pitch control prioritises speed and height control
    % A weighting of 1 provides equal priority (this is the normal mode of operation)
    % A SKE_weighting of 0 provides 100% priority to height control. This is used when no airspeed measurement is available
    % A SKE_weighting of 2 provides 100% priority to speed control. This is used when an underspeed condition is detected. In this instance, if airspeed
    % rises above the demanded value, the pitch angle will be increased by the TECS controller.
global dt
global GRAVITY_MSS
global AP_TECS

spdWeight              = AP_TECS.spdWeight;      
SPE_dem                = AP_TECS.SPE_dem;       
SKE_dem                = AP_TECS.SKE_dem;       
SPEdot_dem             = AP_TECS.SPEdot_dem;    
SKEdot_dem             = AP_TECS.SKEdot_dem;    
SPE_est                = AP_TECS.SPE_est;       
SKE_est                = AP_TECS.SKE_est;       
SPEdot                 = AP_TECS.SPEdot;        
integGain              = AP_TECS.integGain;     
pitch_dem              = AP_TECS.pitch_dem;     
PITCHmaxf              = AP_TECS.PITCHmaxf;     
PITCHminf              = AP_TECS.PITCHminf;     
TAS_state              = AP_TECS.TAS_state;     
timeConstant           = AP_TECS.timeConstant;              
ptchDamp               = AP_TECS.ptchDamp ;     
integSEB_state         = AP_TECS.integSEB_state;
pitch_dem_unc          = AP_TECS.pitch_dem_unc; 
vertAccLim             = AP_TECS.vertAccLim;    
last_pitch_dem         = AP_TECS.last_pitch_dem;
SKEdot                 = AP_TECS.SKEdot;        
TASmax                 = AP_TECS.TASmax;        
TASmin                 = AP_TECS.TASmin;        

     SKE_weighting = constrain_value(spdWeight, 0.0, 2.0);
     SPE_weighting = 2.0 - SKE_weighting;

%     % Calculate Specific Energy Balance demand, and error
%      SEB_dem      = SPE_dem * SPE_weighting - SKE_dem * SKE_weighting;
%      SEBdot_dem   = SPEdot_dem * SPE_weighting - SKEdot_dem * SKE_weighting;
%      SEB_error    = SEB_dem - (SPE_est * SPE_weighting - SKE_est * SKE_weighting);
%      SEBdot_error = SEBdot_dem - (SPEdot * SPE_weighting - SKEdot * SKE_weighting);
%20200421 g byc

%%%%%%%%%
      SPE_err_max = -(0.5  * TASmin * TASmin - SKE_dem);
      SPE_err_min = -(0.5  * TASmax * TASmax - SKE_dem);
    
%      SEB_dem      = SPE_dem * SPE_weighting - SKE_dem * SKE_weighting;
     SEBdot_dem   = SPEdot_dem * SPE_weighting - SKEdot_dem * SKE_weighting;
     SEB_error    = constrain_value((SPE_dem - SPE_est), SPE_err_min, SPE_err_max)*SPE_weighting -( SKE_dem - SKE_est)*SKE_weighting;
     SEBdot_error = SEBdot_dem - (SPEdot * SPE_weighting - SKEdot * SKE_weighting);
%%%%%%%%%%%%%%%%%% 
    
    % Calculate integrator state, constraining input if pitch limits are exceeded
     integSEB_input = SEB_error * integGain;
    if (pitch_dem > PITCHmaxf)
     
        integSEB_input = min(integSEB_input, PITCHmaxf - pitch_dem);
     
    elseif (pitch_dem < PITCHminf)
     
        integSEB_input = max(integSEB_input, PITCHminf - pitch_dem);
    end
     integSEB_delta = integSEB_input * dt;

    % Apply max and min values for integrator state that will allow for no more than
    % 5deg of saturation. This allows for some pitch variation due to gusts before the
    % integrator is clipped. Otherwise the effectiveness of the integrator will be reduced in turbulence
    % During climbout/takeoff, bias the demanded pitch angle so that zero speed error produces a pitch angle
    % demand equal to the minimum value (which is )set by the mission plan during this mode). Otherwise the
    % integrator has to catch up before the nose can be raised to reduce speed during climbout.
    % During flare a different damping gain is used
    %                                                                                                                                                                   
     gainInv = (TAS_state * timeConstant* GRAVITY_MSS);
     temp = SEB_error + SEBdot_dem * timeConstant;
     pitch_damp = ptchDamp;
     temp =temp+ SEBdot_error * pitch_damp;
     
     integSEB_min = (gainInv * (PITCHminf - 0.0783)) - temp;
     integSEB_max = (gainInv * (PITCHmaxf + 0.0783)) - temp;
     integSEB_range = integSEB_max - integSEB_min;

    
    % don't allow the integrator to rise by more than 20% of its full
    % range in one step. This prevents single value glitches from
    % causing massive integrator changes. See Issue#4066
    integSEB_delta = constrain_value(integSEB_delta, -integSEB_range*0.1, integSEB_range*0.1);

    % prevent the constraint on pitch integrator _integSEB_state from
    % itself injecting step changes in the variable. We only want the
    % constraint to prevent large changes due to integSEB_delta, not
    % to cause step changes due to a change in the constrain
    % limits. Large steps in _integSEB_state can cause long term
    % pitch changes
    integSEB_min = min(integSEB_min, integSEB_state);
    integSEB_max = max(integSEB_max, integSEB_state);

    % integrate
    integSEB_state = constrain_value(integSEB_state + integSEB_delta, integSEB_min, integSEB_max);

    % Calculate pitch demand from specific energy balance signals
    pitch_dem_unc = (temp + integSEB_state) / gainInv;

    % Constrain pitch demand
    pitch_dem = constrain_value(pitch_dem_unc, PITCHminf, PITCHmaxf);

    % Rate limit the pitch demand to comply with specified vertical
    % acceleration limit
     ptchRateIncr = dt * vertAccLim / TAS_state;

    if ((pitch_dem - last_pitch_dem) > ptchRateIncr)
     
        pitch_dem = last_pitch_dem + ptchRateIncr;
     
    elseif ((pitch_dem - last_pitch_dem) < -ptchRateIncr)
     
        pitch_dem = last_pitch_dem - ptchRateIncr;
    end

    % re-constrain pitch demand
    pitch_dem = constrain_value(pitch_dem, PITCHminf, PITCHmaxf);

    last_pitch_dem = pitch_dem;
    
    AP_TECS.spdWeight              = spdWeight;
    AP_TECS.SPE_dem                = SPE_dem;
    AP_TECS.SKE_dem                = SKE_dem;
    AP_TECS.SPEdot_dem             = SPEdot_dem;
    AP_TECS.SKEdot_dem             = SKEdot_dem;
    AP_TECS.SPE_est                = SPE_est;
    AP_TECS.SKE_est                = SKE_est;
    AP_TECS.SPEdot                 = SPEdot;
    AP_TECS.integGain              = integGain;
    AP_TECS.pitch_dem              = pitch_dem;
    AP_TECS.PITCHmaxf              = PITCHmaxf;
    AP_TECS.PITCHminf              = PITCHminf;
    AP_TECS.TAS_state              = TAS_state;
    AP_TECS.timeConstant           = timeConstant;
    AP_TECS.ptchDamp               = ptchDamp ;
    AP_TECS.integSEB_state         = integSEB_state;
    AP_TECS.pitch_dem_unc          = pitch_dem_unc;
    AP_TECS.vertAccLim             = vertAccLim;
    AP_TECS.last_pitch_dem         = last_pitch_dem;
    AP_TECS.SKEdot                 = SKEdot;
    AP_TECS.TASmax                 = TASmax;
    AP_TECS.TASmin                 = TASmin;


end

