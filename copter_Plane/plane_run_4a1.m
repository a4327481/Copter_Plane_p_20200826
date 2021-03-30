function plane_run_4a1()
%plane
global Plane
global SRV_Channel
global Copter_Plane

hgt_dem_cm                            = Copter_Plane.hgt_dem_cm;
EAS_dem_cm                            = Copter_Plane.EAS_dem_cm;
aerodynamic_load_factor               = Plane.aerodynamic_load_factor;
k_throttle                            = SRV_Channel.k_throttle;

        update_50hz();
        update_pitch_throttle(  hgt_dem_cm,EAS_dem_cm,aerodynamic_load_factor)
        calc_nav_pitch();
        calc_nav_roll()
        calc_throttle()
        stabilize()
        if(k_throttle>=0.8)
            k_throttle=0.8;
        end
        output_to_motors_plane_4a1();
%         k_flap=k_aileron*p_flap_plane;


Copter_Plane.hgt_dem_cm                            = hgt_dem_cm;
Copter_Plane.EAS_dem_cm                            = EAS_dem_cm;    
Plane.aerodynamic_load_factor                      = aerodynamic_load_factor;
SRV_Channel.k_throttle                             = k_throttle;  
  
end

