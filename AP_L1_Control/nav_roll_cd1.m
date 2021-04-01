function    ret=nav_roll_cd1( )  
 global HD
 global GRAVITY_MSS
 global AP_L1
 global SINS
 latAccDem          = AP_L1.latAccDem;
 pitch              = SINS.pitch;
    ret = cos(pitch)*atan(latAccDem /GRAVITY_MSS ) * 100.0*HD; 
    ret = constrain_value(ret, -9000, 9000);
  
end

