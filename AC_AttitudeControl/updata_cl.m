function updata_cl()
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
global GRAVITY_MSS
global SINS
accel_x   = SINS.accel_x;
accel_y   = SINS.accel_y;
accel_z   = SINS.accel_z;
roll      = SINS.roll;
pitch     = SINS.pitch;
yaw       = SINS.yaw;

 qtemp=from_euler([roll pitch yaw]);
 rot_body_to_ned=rotation_matrix(qtemp);
 accel_ef=rot_body_to_ned*[accel_x accel_y accel_z]';
z_accel_meas= -(accel_ef(3)+GRAVITY_MSS)*100;
 

SINS.rot_body_to_ned  = rot_body_to_ned;
SINS.z_accel_meas     = z_accel_meas;
end

