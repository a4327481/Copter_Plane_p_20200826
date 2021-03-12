function calc_leash_length_xy()
% calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration
%     should be called whenever the speed, acceleration or position kP is modified
%     // todo: remove _flags.recalc_leash_xy or don't call this function after each variable change.
 global AC_PosControl

speed_cms         =AC_PosControl.speed_cms;
accel_cms         =AC_PosControl.accel_cms;
p_pos_xy          =AC_PosControl.p_pos_xy;
leash             = AC_PosControl.leash;
%     leash = calc_leash_length(POSCONTROL_SPEED, POSCONTROL_ACCEL_XY,POSCONTROL_POS_XY_P);
%     if ( recalc_leash_xy)  
%             leash = calc_leash_length(POSCONTROL_SPEED, POSCONTROL_ACCEL_XY,POSCONTROL_POS_XY_P);
%             recalc_leash_xy = 0;
%     end
        leash = calc_leash_length(speed_cms, accel_cms, p_pos_xy);

%      if (_flags.recalc_leash_xy) 
%         _leash = calc_leash_length(_speed_cms, _accel_cms, _p_pos_xy.kP());
%         _flags.recalc_leash_xy = false;
 AC_PosControl.leash               = leash;
 
end

