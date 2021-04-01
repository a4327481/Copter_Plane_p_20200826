function  update_energies( )
 
global GRAVITY_MSS
global SINS
global AP_TECS



SPE_dem                 = AP_TECS.SPE_dem;
hgt_dem_adj             = AP_TECS.hgt_dem_adj;
TAS_dem_adj             = AP_TECS.TAS_dem_adj;
hgt_rate_dem            = AP_TECS.hgt_rate_dem;
TAS_state               = AP_TECS.TAS_state;
TAS_rate_dem            = AP_TECS.TAS_rate_dem;
SKE_dem                 = AP_TECS.SKE_dem;
SPEdot_dem              = AP_TECS.SPEdot_dem;
SKEdot_dem              = AP_TECS.SKEdot_dem;
SPE_est                 = AP_TECS.SPE_est;
SKE_est                 = AP_TECS.SKE_est;
SPEdot                  = AP_TECS.SPEdot;
SKEdot                  = AP_TECS.SKEdot;
climb_rate              = AP_TECS.climb_rate;
vel_dot                 = AP_TECS.vel_dot;
height                  = SINS.curr_alt/100;
    % Calculate specific energy demands
    SPE_dem = hgt_dem_adj * GRAVITY_MSS;
    SKE_dem = 0.5 * TAS_dem_adj * TAS_dem_adj;

    % Calculate specific energy rate demands
    SPEdot_dem = hgt_rate_dem * GRAVITY_MSS;
    SKEdot_dem = TAS_state * TAS_rate_dem;

    % Calculate specific energy
    SPE_est = height * GRAVITY_MSS;
    SKE_est = 0.5 * TAS_state * TAS_state;

    % Calculate specific energy rate
    SPEdot = climb_rate * GRAVITY_MSS;
    SKEdot = TAS_state * vel_dot;

 AP_TECS.SPE_dem                =   SPE_dem;                 
 AP_TECS.hgt_dem_adj            =   hgt_dem_adj;             
 AP_TECS.TAS_dem_adj            =   TAS_dem_adj;             
 AP_TECS.hgt_rate_dem           =   hgt_rate_dem;            
 AP_TECS.TAS_state              =   TAS_state;               
 AP_TECS.TAS_rate_dem           =   TAS_rate_dem;            
 AP_TECS.SKE_dem                =   SKE_dem;                 
 AP_TECS.SPEdot_dem             =   SPEdot_dem;              
 AP_TECS.SKEdot_dem             =   SKEdot_dem;              
 AP_TECS.SPE_est                =   SPE_est;                 
 AP_TECS.SKE_est                =   SKE_est;                 
 AP_TECS.SPEdot                 =   SPEdot;                  
 AP_TECS.SKEdot                 =   SKEdot;                  
 AP_TECS.climb_rate             =   climb_rate;              
 AP_TECS.vel_dot                =   vel_dot;                 


end

