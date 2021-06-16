function plane_run()
%plane

global Copter_Plane
global Plane
global SRV_Channel
global plane_mode

hgt_dem_cm                            = Copter_Plane.hgt_dem_cm;
EAS_dem_cm                            = Copter_Plane.EAS_dem_cm;
aerodynamic_load_factor               = Plane.aerodynamic_load_factor;

update_50hz();
update_pitch_throttle(  hgt_dem_cm,EAS_dem_cm,aerodynamic_load_factor)
calc_nav_pitch();
calc_nav_roll();
calc_throttle();
stabilize();
switch plane_mode
    case {ENUM_plane_mode.V10 ,ENUM_plane_mode.V10_1}
        if(SRV_Channel.k_throttle>0.8)
            SRV_Channel.k_throttle=0.8;
        end     
end
AP_Motors_output();
end

