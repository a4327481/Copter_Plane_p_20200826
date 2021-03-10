function  calc_leash_length_z()
global AC_PosControl

p_pos_z                 = AC_PosControl.p_pos_z;
accel_z_cms             = AC_PosControl.accel_z_cms;
speed_up_cms            = AC_PosControl.speed_up_cms;
speed_down_cms          = AC_PosControl.speed_down_cms;

flags_recalc_leash_z    = AC_PosControl.flags_recalc_leash_z;
leash_up_z              = AC_PosControl.leash_up_z;
leash_down_z            = AC_PosControl.leash_down_z;
    
    if (flags_recalc_leash_z)
        leash_up_z = calc_leash_length(speed_up_cms, accel_z_cms, p_pos_z);
        leash_down_z = calc_leash_length(-speed_down_cms, accel_z_cms, p_pos_z);
        flags_recalc_leash_z = false;
    end
AC_PosControl.flags_recalc_leash_z      = flags_recalc_leash_z;
AC_PosControl.leash_up_z                = leash_up_z;
AC_PosControl.leash_down_z              = leash_down_z;    
    
end

