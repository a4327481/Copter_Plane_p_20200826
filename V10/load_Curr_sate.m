function load_Curr_sate()

global mode
global accel_x
global accel_y
global accel_z
global gyro_x
global gyro_y
global gyro_z
global aspeed
global EAS2TAS
global groundspeed_vector
global height
global Vz
global curr_vel
global curr_alt
global current_loc
global roll
global pitch
global yaw
global PathModeOut_sl
global Curr_sate
global algo_dbg_param
global HD
 accel_x=Curr_sate.accel_x;
 accel_y=Curr_sate.accel_y;
 accel_z=Curr_sate.accel_z;
 gyro_x=Curr_sate.gyro_x;
 gyro_y=Curr_sate.gyro_y;
 gyro_z=Curr_sate.gyro_z;
 aspeed=Curr_sate.EAS_Algo;
 EAS2TAS=Curr_sate.EAS2TAS_Algo;
 groundspeed_vector=Curr_sate.curVelNED(1:2);
 Vz=Curr_sate.curVelNED(3);
 curr_vel=[Curr_sate.curVelNED(1) Curr_sate.curVelNED(2) -Curr_sate.curVelNED(3)]*100;
%  curr_alt=Curr_sate.curLLA(3)*100;
%  height=Curr_sate.curLLA(3);
 current_loc=Curr_sate.curLLA(1:2)*1e7;
 pitch=Curr_sate.pitchd/HD;
 roll=Curr_sate.rolld/HD;
 yaw=Curr_sate.yawd/HD;
 
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
    height                                  =Curr_sate.curLLA(3);
 else
    PathModeOut_sl.headingCmd               =algo_dbg_param.headingCmd;
    PathModeOut_sl.groundspeedCmd           =algo_dbg_param.groundspeedCmd;
    PathModeOut_sl.heightCmd                =algo_dbg_param.heightCmd;
    PathModeOut_sl.flightTaskMode           =algo_dbg_param.flightTaskMode;
    PathModeOut_sl.maxClimbSpeed            =algo_dbg_param.maxClimbSpeed;  
    curr_alt                                =Curr_sate.NAV_alt*100;
    height                                  =Curr_sate.NAV_alt;
 end
 
end

