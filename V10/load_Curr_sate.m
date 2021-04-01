function load_Curr_sate()

global Copter_Plane
global SINS



global PathModeOut_sl
global Curr_sate
global algo_dbg_param
global HD
 mode  = Copter_Plane.mode;
 accel_x=Curr_sate.accel_x;
 accel_y=Curr_sate.accel_y;
 accel_z=Curr_sate.accel_z;
 gyro_x=Curr_sate.gyro_x;
 gyro_y=Curr_sate.gyro_y;
 gyro_z=Curr_sate.gyro_z;
 roll=Curr_sate.rolld/HD;
 pitch=Curr_sate.pitchd/HD;
 yaw=Curr_sate.yawd/HD;
 aspeed=Curr_sate.EAS_Algo;
 EAS2TAS=Curr_sate.EAS2TAS_Algo;
 if(EAS2TAS<0.3)
     EAS2TAS=0.3;
 end
 groundspeed_vector=Curr_sate.curVelNED(1:2);
 curr_vel=[Curr_sate.curVelNED(1) Curr_sate.curVelNED(2) -Curr_sate.curVelNED(3)]*100;
 curr_loc=Curr_sate.curLLA(1:2)*1e7;
 if(mode==10)
    PathModeOut_sl.headingCmd               =Curr_sate.PathModeOut.headingCmd;
    PathModeOut_sl.groundspeedCmd           =Curr_sate.PathModeOut.groundspeedCmd;
    PathModeOut_sl.heightCmd                =Curr_sate.PathModeOut.heightCmd*100;
    PathModeOut_sl.flightTaskMode           =Curr_sate.PathModeOut.flightTaskMode;
    PathModeOut_sl.flightControlMode        =Curr_sate.PathModeOut.flightControlMode;
    PathModeOut_sl.maxClimbSpeed            =Curr_sate.PathModeOut.maxClimbSpeed*100;
    PathModeOut_sl.turnCenterLL             =Curr_sate.PathModeOut.turnCenterLL*1e7;
    PathModeOut_sl.prePathPoint_LLA         =Curr_sate.PathModeOut.prePathPoint_LLA*1e7;
    PathModeOut_sl.curPathPoint_LLA         =Curr_sate.PathModeOut.curPathPoint_LLA*1e7;
    PathModeOut_sl.rollCmd                  =Curr_sate.PathModeOut.rollCmd;   
    curr_alt                                =Curr_sate.curLLA(3)*100;
 else
    PathModeOut_sl.headingCmd               =algo_dbg_param.headingCmd;
    PathModeOut_sl.groundspeedCmd           =algo_dbg_param.groundspeedCmd;
    PathModeOut_sl.heightCmd                =algo_dbg_param.heightCmd;
    PathModeOut_sl.flightTaskMode           =algo_dbg_param.flightTaskMode;
    PathModeOut_sl.maxClimbSpeed            =algo_dbg_param.maxClimbSpeed;  
    curr_alt                                =Curr_sate.NAV_alt*100;
 end
 
SINS.accel_x                 = accel_x;
SINS.accel_y                 = accel_y;
SINS.accel_z                 = accel_z;
SINS.gyro_x                  = gyro_x;
SINS.gyro_y                  = gyro_y;
SINS.gyro_z                  = gyro_z;
SINS.roll                    = roll;
SINS.pitch                   = pitch;
SINS.yaw                     = yaw;
SINS.aspeed                  = aspeed;
SINS.EAS2TAS                 = EAS2TAS;
SINS.groundspeed_vector      = groundspeed_vector;
SINS.curr_vel                = curr_vel;
SINS.curr_alt                = curr_alt;
SINS.curr_loc                = curr_loc;
end

