function plane_run()
%plane
global hgt_dem_cm
global EAS_dem_cm
global Plane

aerodynamic_load_factor               = Plane.aerodynamic_load_factor;

        update_50hz();
        update_pitch_throttle(  hgt_dem_cm,EAS_dem_cm,aerodynamic_load_factor)
        calc_nav_pitch();
        calc_nav_roll()
        calc_throttle()
        stabilize()
        output_to_motors_plane();
        
Plane.aerodynamic_load_factor               = aerodynamic_load_factor;
       
end

