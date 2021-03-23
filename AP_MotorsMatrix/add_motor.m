function  add_motor(  motor_num,   angle_degrees,   yaw_fac)

global AP_Motors

 roll_factor               = AP_Motors.roll_factor;
 pitch_factor              = AP_Motors.pitch_factor;
 yaw_factor                = AP_Motors.yaw_factor; 
 
roll_factor_in_degrees= angle_degrees;
pitch_factor_in_degrees=angle_degrees;
roll_factor(motor_num)=cos(radians(roll_factor_in_degrees + 90));
pitch_factor(motor_num)=cos(radians(pitch_factor_in_degrees));
yaw_factor(motor_num)=yaw_fac;

 AP_Motors.roll_factor               = roll_factor;
 AP_Motors.pitch_factor              = pitch_factor;
 AP_Motors.yaw_factor                = yaw_factor; 

end

