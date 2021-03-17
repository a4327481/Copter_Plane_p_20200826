function  output=get_althold_lean_angle_max()  
% Return tilt angle limit for pilot input that prioritises altitude hold over lean angle

  global HD
  global AC_Attitude
  
  althold_lean_angle_max                = AC_Attitude.althold_lean_angle_max;
  AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN   = AC_Attitude.AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN;
  
   %convert to centi-degrees for public interface
     output=max((althold_lean_angle_max)*HD, AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN) * 100.0;
     
  AC_Attitude.althold_lean_angle_max                = althold_lean_angle_max;
  AC_Attitude.AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN   = AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN;
 

end

