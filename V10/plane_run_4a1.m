function plane_run_4a1()
%plane
global hgt_dem_cm
global EAS_dem_cm
global aerodynamic_load_factor
global center_WP
global radius
global loiter_direction
        update_50hz();
        update_pitch_throttle(  hgt_dem_cm,EAS_dem_cm,aerodynamic_load_factor)
        calc_nav_pitch();
        calc_nav_roll()
        calc_throttle()
        stabilize()
        output_to_motors_plane_4a1();
end

