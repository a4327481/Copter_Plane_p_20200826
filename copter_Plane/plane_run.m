function plane_run()
%plane

global Copter_Plane
global Plane

hgt_dem_cm                            = Copter_Plane.hgt_dem_cm;
EAS_dem_cm                            = Copter_Plane.EAS_dem_cm;
aerodynamic_load_factor               = Plane.aerodynamic_load_factor;

        update_50hz();
        update_pitch_throttle(  hgt_dem_cm,EAS_dem_cm,aerodynamic_load_factor)
        calc_nav_pitch();
        calc_nav_roll()
        calc_throttle()
        stabilize()
        AP_Motors_output();

       
end

