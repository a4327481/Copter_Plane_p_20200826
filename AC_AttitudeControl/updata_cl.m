function updata_cl()
global GRAVITY_MSS
global SINS
accel_x                                  = SINS.accel_x;
accel_y                                  = SINS.accel_y;
accel_z                                  = SINS.accel_z;
roll                                     = SINS.roll;
pitch                                    = SINS.pitch;
yaw                                      = SINS.yaw;
aspeed                                   = SINS.aspeed;
arspeed_filt                             = Copter_Plane.arspeed_filt;

persistent arspeed_temp
if isempty(arspeed_temp)
    arspeed_temp = aspeed;
end

 qtemp=from_euler([roll pitch yaw]);
 rot_body_to_ned=rotation_matrix(qtemp);
 accel_ef=rot_body_to_ned*[accel_x accel_y accel_z]';
 z_accel_meas= -(accel_ef(3)+GRAVITY_MSS)*100;
 
 arspeed_temp=arspeed_temp+(aspeed-arspeed_temp)* get_filt_alpha(arspeed_filt);
 aspeed=arspeed_temp;
 
SINS.aspeed                              = aspeed;
SINS.rot_body_to_ned                     = rot_body_to_ned;
SINS.z_accel_meas                        = z_accel_meas;
end

